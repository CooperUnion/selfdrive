#ifndef MESSAGE_ID_H
#define MESSAGE_ID_H

//if (rx_frame.MsgID >= ESTOP_RANGE_START && rx_frame.MsgID <= ESTOP_RANGE_END) set queue to canToEStop
//0x00000000
//0x10000000
//0x_____
enum canmsg_ID  {
    ADMIN_RANGE_START   = 0x0000,
    ADMIN_RANGE_END     = 0x03FF,
    ESTOP_RANGE_START   = 0x0400,
        MSG_ESTOP,
    ESTOP_RANGE_END     = 0x07FF,
    HOUSE_RANGE_START   = 0x0800,
    HOUSE_RANGE_END     = 0x0BFF,
    WATCH_RANGE_START   = 0x0C00,
    WATCH_RANGE_END     = 0x0FFF,
    BRK_RANGE_START     = 0x1000,
    BRK_RANGE_END       = 0x2FFF,
    STR_RANGE_START     = 0x3000,
    STR_RANGE_END       = 0x4FFF,
    SPD_RANGE_START     = 0x5000,
    SPD_RANGE_END       = 0x6FFF,
    MSC_RANGE_START     = 0x7000,
    MSC_RANGE_END       = 0x8FFF
};

#endif

