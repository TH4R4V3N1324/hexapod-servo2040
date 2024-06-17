#ifndef _DATAPACKET_H_
#define _DATAPACKET_H_

// Define the data structure with no padding
#pragma pack(push, 1)
struct DataPacket {
    bool Right;
    bool Left;
    bool Up;
    bool Down;

    bool Square;
    bool Cross;
    bool Circle;
    bool Triangle;

    int LStickX;
    int LStickY;

    int RStickX;
    int RStickY;
};
#pragma pack(pop)

extern DataPacket receivedData;

#endif