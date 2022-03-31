#include <stdint.h>
#define DEFAULT_BUS 0x01
#define NONE        0x00
#define BRAKE_ID    0x00
#define ACCEL_ID    0x20
#define STEER_ID    0x40

#define COMMAND_ID 0x160
#define CONTROL_ID 0x60

#define SUBSYS_MASK 10000000


union float_hex_convert{
    unsigned int hex;
    char data[4];
    float val;
};

enum DeviceType{
    KIACAN,
    MCM,
    DEFAULT
};

enum MCM_MESSAGE_TYPE{
    CONTROL_RESPONSE,
    FAULT,
    OVERRIDE
};

typedef struct MCM_STATE{
    bool control_State;
    bool Brake_Control_State;
    bool Accel_Control_State;
    bool Steer_Control_State;
    int FAULT;
}MCM_STATE;

//STRUCTS FOR CAN DATA Frames
namespace CanMessage
{

typedef struct MCM_DATA{
    MCM_MESSAGE_TYPE type;
    unsigned int subsys_id;
    char hex_id;
    float float_data;
    bool bool_data;
}MCM_DATA;


typedef struct WHL_SPD{
    float FL;
    float FR;
    float RL;
    float RR;
}WHL_SPD;

} //namespace Can Message
