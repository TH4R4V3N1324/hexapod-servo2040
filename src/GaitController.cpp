#include "GaitController.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "ConfigManager.h"
#include "RobotConfig.h"

using gait_core::Scalar;
using gait_core::NUM_LEGS;

// The wire values of Gait and Mode are passed straight to the engine
static_assert(static_cast<int>(::GAIT_TRIPOD) == gait_core::GAIT_TRIPOD &&
              static_cast<int>(::GAIT_RIPPLE) == gait_core::GAIT_RIPPLE &&
              static_cast<int>(::GAIT_WAVE) == gait_core::GAIT_WAVE &&
              static_cast<int>(::NUM_GAITS) == gait_core::NUM_GAITS, "Gait values must match gait_core");
static_assert(static_cast<int>(::MODE_NORMAL) == gait_core::MODE_NORMAL &&
              static_cast<int>(::MODE_STRAFE) == gait_core::MODE_STRAFE &&
              static_cast<int>(::MODE_TILT) == gait_core::MODE_TILT &&
              static_cast<int>(::MODE_CONFIG) == gait_core::NUM_MODES, "Mode values must match gait_core");

namespace {

/**
 * @brief A joystick value as -1 to 1, with the dead zone removed
*/
Scalar stick(int16_t value) {
    if (std::abs(value) <= robot::JOYSTICK_DEADZONE) {
        return 0;
    }
    return std::clamp(static_cast<Scalar>(value) / robot::JOYSTICK_FULL_SCALE, Scalar(-1), Scalar(1));
}

/**
 * @brief The GaitParams switch for a protocol feature (all "live": they apply while running)
*/
bool& featureFlag(gait_core::GaitParams& p, GaitFeature f) {
    switch (f) {
        case FEATURE_WALK_LEVELING: return p.walk_leveling;
        case FEATURE_LOAD_BALANCING: return p.load_balancing;
        case FEATURE_TILT_LEVELING: return p.tilt_leveling;
        case FEATURE_TERRAIN_ADAPTATION:
        default: return p.terrain_adaptation;
    }
}

bool featureFlag(const gait_core::GaitParams& p, GaitFeature f) {
    return featureFlag(const_cast<gait_core::GaitParams&>(p), f);
}

// Features that need servo position or load feedback, which hobby servos don't give
bool needsFeedback(GaitFeature f) {
    return f == FEATURE_TERRAIN_ADAPTATION || f == FEATURE_LOAD_BALANCING;
}

const char* featureName(GaitFeature f) {
    switch (f) {
        case FEATURE_TERRAIN_ADAPTATION: return "terrain adaptation";
        case FEATURE_WALK_LEVELING: return "walk leveling";
        case FEATURE_LOAD_BALANCING: return "load balancing";
        case FEATURE_TILT_LEVELING: return "tilt leveling";
        default: return "?";
    }
}

const char* levelName(gait_core::LogLevel level) {
    switch (level) {
        case gait_core::LogLevel::Debug: return "DEBUG";
        case gait_core::LogLevel::Info: return "INFO";
        case gait_core::LogLevel::Warn: return "WARN";
        case gait_core::LogLevel::Error: return "ERROR";
    }
    return "";
}

}  // namespace

GaitController::GaitController(Servos& servos)
    : servos(servos), engine(robot::gaitParams(), robot::geometry()), battery(1, robot::batteryParams()) {
    engine.log = [](gait_core::LogLevel level, const char* message) {
        printf("[gait %s] %s\n", levelName(level), message);
    };
    engine.seatedChanged = [](bool seated) {
        printf("[gait] %s\n", seated ? "seated" : "standing");
    };
    battery.log = [](gait_core::LogLevel level, const char* message) {
        printf("[battery %s] %s\n", levelName(level), message);
    };
}

void GaitController::begin() {
    seatLegs();
}

/**
 * @brief Put each leg in the seated pose in turn, starting its servos, and tell the engine that's
 *        where they are
 * @note Blocks for NUM_LEGS * BOOT_LEG_STAGGER_MS. The joint state sent here is the only one the
 *       engine gets: from here on it runs open loop.
*/
void GaitController::seatLegs() {
    const gait_core::GaitParams& p = engine.params();
    const gait_core::KinematicSolver ik(robot::geometry());
    const gait_core::JointAngles seated = ik.solveIK(gait_core::Vec3(p.sit_reach, 0, -p.sit_height));

    gait_core::JointFeedback feedback;
    for (int legNum = 0; legNum < NUM_LEGS; ++legNum) {
        servoOutput[legNum] = Servos::toServoDegrees(legNum, seated);
        servos.write(legNum, servoOutput[legNum]);
        sleep_ms(robot::BOOT_LEG_STAGGER_MS);
        feedback.angles[legNum] = seated;
        feedback.present[legNum] = true;
    }
    feedback.stamp = static_cast<gait_core::Timestamp>(time_us_64()) * 1000;
    engine.onJointStates(feedback);
    servosPowered = true;
}

/**
 * @brief Start or stop the servos when the engine's servo power changes
 * @note The engine turns power on with a stand command and waits for joint states; the legs have been
 *       limp meanwhile, so they're put back in the seated pose first, as at boot.
*/
void GaitController::followServoPower(bool power) {
    if (power && !servosPowered) {
        printf("[servos] on\n");
        seatLegs();
    } else if (!power && servosPowered) {
        servos.disableAll();
        servosPowered = false;
        configMode = false;
        blending = false;
        printf("[servos] off: send a stand command to start them again\n");
    }
}

/**
 * @brief Act on the battery monitor's decisions: sit down, and stop the servos, at critical
*/
void GaitController::updateBattery(uint64_t nowUs) {
    if (!robot::BATTERY_MONITORING) {
        return;
    }
    const hexapod_behaviour::BatteryStatus status = battery.update(static_cast<gait_core::Timestamp>(nowUs) * 1000);
    if (status.sit) {
        const gait_core::CommandResult result = status.powerOff ? engine.powerDown() : engine.sit(true);
        if (!result.success) {
            printf("[battery] couldn't sit down: %s\n", result.message);
        }
        battery.sitRequested();
    }
}

void GaitController::onPower(float voltage, float current, uint64_t nowUs) {
    // BatteryMonitor counts discharge current as negative
    battery.onReading(0, voltage, -current, static_cast<gait_core::Timestamp>(nowUs) * 1000);
}

void GaitController::handleCommand(const ControlPacket& packet) {
    const int16_t* args = packet.commandArgs;
    switch (packet.command) {
        case CMD_SET_GAIT:
            if (args[0] >= 0 && args[0] < NUM_GAITS) {
                engine.setGait(static_cast<uint8_t>(args[0]));
            } else {
                printf("Ignored invalid gait %d\n", args[0]);
            }
            break;

        case CMD_SET_MODE:
            if (args[0] == MODE_CONFIG) {
                if (!servosPowered) {
                    printf("Ignored MODE_CONFIG: the servos are off\n");
                } else if (!configMode) {
                    // Out of tilt mode first, so the engine eases the body back to neutral meanwhile
                    engine.setMode(gait_core::MODE_NORMAL);
                    configMode = true;
                    blending = true;
                }
            } else if (args[0] >= 0 && args[0] < MODE_CONFIG) {
                if (engine.setMode(static_cast<uint8_t>(args[0])).success && configMode) {
                    configMode = false;
                    blending = true;
                }
            } else {
                printf("Ignored invalid mode %d\n", args[0]);
            }
            break;

        case CMD_SET_CONFIG:
            if (args[0] >= 0 && args[0] < NUM_LEGS && args[1] >= 0 && args[1] < 3) {
                configManager.SetLegConfig(args[0], args[1], args[2]);
            } else {
                printf("Ignored calibration for leg %d joint %d\n", args[0], args[1]);
            }
            break;

        case CMD_HOME_STANCE:
            engine.sit(false);
            break;

        case CMD_REQUEST_CONFIG:
            if (args[0] >= 0 && args[0] < NUM_LEGS) {
                requestedConfigLeg = args[0];
            }
            break;

        case CMD_SIT:
            engine.sit(args[0] != 0);
            break;

        case CMD_POWER_DOWN:
            engine.powerDown();
            break;

        case CMD_SET_FEATURE:
            if (args[0] < 0 || args[0] >= NUM_FEATURES) {
                printf("Ignored invalid feature %d\n", args[0]);
            } else if (args[1] != 0 && needsFeedback(static_cast<GaitFeature>(args[0]))) {
                // Hobby servos report nothing, so these can't work (RobotConfig.h keeps them off)
                printf("Refused to turn on %s: the servos give no feedback\n",
                       featureName(static_cast<GaitFeature>(args[0])));
            } else {
                featureFlag(engine.params(), static_cast<GaitFeature>(args[0])) = args[1] != 0;
                printf("[gait] %s %s\n", featureName(static_cast<GaitFeature>(args[0])), args[1] != 0 ? "on" : "off");
            }
            break;

        case CMD_NONE:
        default:
            break;
    }
}

/**
 * @brief Turn the sticks into a velocity command (or a body pose in tilt mode)
 * @note Sticks: X positive is right, Y positive is forward. Left stick drives, right stick X turns.
 *       In normal mode the engine turns with the left stick's
 *       X instead of strafing. In tilt mode the left stick shifts the body, the right stick
 *       rolls and pitches it, tiltYaw yaws it and tiltZ raises and lowers it.
*/
void GaitController::applySticks(const ControlPacket& packet, bool commsOk) {
    Scalar lx = 0, ly = 0, rx = 0, ry = 0, yaw = 0, z = 0;
    if (commsOk && !configMode) {
        lx = stick(packet.joystick1X);
        ly = stick(packet.joystick1Y);
        rx = stick(packet.joystick2X);
        ry = stick(packet.joystick2Y);
        yaw = stick(packet.tiltYaw);
        z = stick(packet.tiltZ);
    }

    gait_core::VelocityCommand command{};
    if (engine.gaitConfig().currentMode == gait_core::MODE_TILT) {
        const gait_core::GaitParams& p = engine.params();
        engine.setBodyPose(rx * p.tilt_max_angle,     // stick right lowers the right side
                           ry * p.tilt_max_angle,     // stick forward lowers the nose
                           yaw * p.tilt_max_angle,    // positive turns left
                           gait_core::Vec3(ly, -lx, z) * p.tilt_max_translation);
    } else {
        // Full stick is the gait's top speed
        const Scalar maxLinear = engine.maxFootSpeed();
        const Scalar maxAngular = maxLinear / engine.trajectory().maxBodyRadius();
        command.vx = ly * maxLinear;
        command.vy = -lx * maxLinear;
        command.wz = -rx * maxAngular;
    }

    // Smoothed; a stop goes straight through
    if (command.vx == 0 && command.vy == 0 && command.wz == 0) {
        filteredVelocity = command;
    } else {
        const Scalar a = robot::VELOCITY_SMOOTHING;
        filteredVelocity.vx += a * (command.vx - filteredVelocity.vx);
        filteredVelocity.vy += a * (command.vy - filteredVelocity.vy);
        filteredVelocity.wz += a * (command.wz - filteredVelocity.wz);
    }
    engine.setVelocity(filteredVelocity);
}

/**
 * @brief Body height from the controller (mm), passed on whenever it changes
 * @note 0 means not set. Asked again once standing if it changed while seated, since the engine
 *       refuses height changes while sitting. Out of range values are refused by the engine.
*/
void GaitController::applyHeight(int16_t heightMm) {
    if (heightMm == lastHeightRequest || heightMm <= 0 || !engine.initialized() || engine.seated()) {
        return;
    }
    lastHeightRequest = heightMm;
    engine.setHeight(-static_cast<Scalar>(heightMm) / 1000);
}

void GaitController::update(Scalar dt, uint64_t nowUs, const ControlPacket& packet, bool commsOk) {
    applySticks(packet, commsOk);
    if (commsOk) {
        applyHeight(packet.currentHeight);
    }
    updateBattery(nowUs);
    const gait_core::GaitOutput output = engine.update(dt, static_cast<gait_core::Timestamp>(nowUs) * 1000);
    followServoPower(output.servoPower);
    if (servosPowered) {
        writeServos(output, dt);
    }
}

void GaitController::onImu(float roll, float pitch, float yaw, uint64_t nowUs) {
    gait_core::ImuSample sample;
    sample.roll = roll;
    sample.pitch = pitch;
    sample.yaw = yaw;
    sample.stamp = static_cast<gait_core::Timestamp>(nowUs) * 1000;
    engine.setImu(sample);
}

/**
 * @brief Write the engine's joint targets, or the calibration pose in MODE_CONFIG
 * @note Moving into or out of the calibration pose, the servos slew there at CONFIG_SLEW_DEG_PER_S
 *       rather than jumping. Before the engine's first output the servos hold where they are.
*/
void GaitController::writeServos(const gait_core::GaitOutput& output, Scalar dt) {
    std::array<ServoDegrees, NUM_LEGS> target = servoOutput;
    if (configMode) {
        for (auto& leg : target) {
            leg = {0, 0, 0};
        }
    } else if (output.commanded) {
        for (int legNum = 0; legNum < NUM_LEGS; ++legNum) {
            target[legNum] = Servos::toServoDegrees(legNum, output.joints[legNum]);
        }
    }

    if (blending) {
        const float maxStep = robot::CONFIG_SLEW_DEG_PER_S * dt;
        bool arrived = true;
        for (int legNum = 0; legNum < NUM_LEGS; ++legNum) {
            for (int joint = 0; joint < 3; ++joint) {
                float& value = servoOutput[legNum][joint];
                const float error = target[legNum][joint] - value;
                value += std::clamp(error, -maxStep, maxStep);
                arrived = arrived && std::abs(error) <= maxStep;
            }
        }
        blending = !arrived;
    } else {
        servoOutput = target;
    }

    for (int legNum = 0; legNum < NUM_LEGS; ++legNum) {
        servos.write(legNum, servoOutput[legNum]);
    }
}

void GaitController::fillTelemetry(HexPacket& packet) const {
    const gait_core::GaitConfig& config = engine.gaitConfig();
    memcpy(packet.legConfigs, configManager.getLegOffsets(requestedConfigLeg), sizeof(packet.legConfigs));
    packet.version = HEXAPOD_PROTOCOL_VERSION;
    packet.currentHeight = static_cast<int16_t>(std::lround(-config.currentHeight * 1000));
    packet.currentGait = static_cast<Gait>(config.gaitChangeRequested ? config.pendingGait : config.currentGait);
    packet.currentMode = configMode ? MODE_CONFIG : static_cast<Mode>(config.currentMode);

    uint8_t flags = 0;
    if (engine.seated()) flags |= HEX_FLAG_SEATED;
    if (engine.servoPower()) flags |= HEX_FLAG_SERVO_POWER;  // the PWM signals are on
    for (int f = 0; f < NUM_FEATURES; ++f) {
        if (featureFlag(engine.params(), static_cast<GaitFeature>(f))) flags |= HEX_FLAG_FEATURE(f);
    }
    packet.flags = flags;
}
