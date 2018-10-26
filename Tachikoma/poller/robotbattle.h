#ifndef ROBOTBATTLE_H
#define ROBOTBATTLE_H

#include "my_functions.h"

#define ROBOT_SIZE 33.0
#define MISSILE_SIZE 3
#define MAX_SPEED 25
#define FIELD_SIZE 16380
#define DEG_TO_RADIAN 0.0174532925f
#define RADIAN_TO_DEG 57.2957795130823f
#define MISSILE_SPEED 44
#define MAX_ACCEL_SPEED 2
#define MAX_DECEL_SPEED 25

#define MAX_BODY_ROTATE 5
#define MAX_GUN_ROTATE 10
#define MAX_RADAR_ROTATE 15

#define ROTATE_BODY 1
#define ROTATE_GUN 2
#define ROTATE_RADAR 4
#define ROTATE_LOCK_BODY 1
#define ROTATE_LOCK_RADAR 2

//taken from http://www.pixelbeat.org/programming/gcc/static_assert.html
//#define ct_assert(e) {enum { ct_assert_value = 1/(!!(e)) };}

enum SystemVars
{
	_true,
	_false,
	_result,
	_name,
	_gunheat,
	_energy,
	_bodyaim,
	_radaraim,
	_gunaim,
	_accel,
	_velocity,
	_speedtarget,
	_rotating,
	_moving,
	_xpos,
	_ypos,
	_movermn,
	_bodyrmn,
	_radarrmn,
	_gunrmn,
	_cldbearing,
    _cldheading,
    _dtcheading,
    _dtcbearing,
    _dtcenergy,
    _dtcdistance,
    _ping,
    _deceltarget,
    _accelltarget,
	_internal_result,
	SYSTEM_VAR_COUNT
};

//events are in order of priority
enum EventEnum
{
    EVENT_INIT,
    EVENT_DEAD,
    EVENT_CORE,
    EVENT_MISSILECOLLISION,
    EVENT_ROBOTCOLLISION,
    EVENT_WALLCOLLISION,
    EVENT_ROBOTDETECT,
    EVENT_WALLDETECT,
    EVENT_PING,
    EVENT_AUTOSCAN,
    MAX_EVENTS
};

enum StateEnum
{
	PARSE_SWAP_STATE,
	CHECK_INDENT,
	PARSE_ERROR,
    CMD_ERROR,
	CHECK_COMMENT,
	NEXT_LINE,
	PARSE_COMMAND,
	PARSE_FUNCTION,
	PARSE_FUNCTION_START,
	PARSE_PARAMS,
	PARSE_PARAMS_MATH,
	KILLED,
	WAIT_ON_MOTION,
	WAIT_ON_ROTATION,
        WAITFOR_STATE,
	IDLE_STATE,
	NEXT_COMMAND,
	NEXT_EVENT,
	STALLED,
	PARSE_DONE,
	PARSE_ENDBLOCK,
	PARSE_FUNCTION_END,
	COMPILE_COMMAND,
	PARSE_ASSIGNMENT,
	PARSE_MATH,
	PARSE_MATH_ANGLES,
	PARSE_MATH_ADDSUB,
	PARSE_MATH_MULDIV,
	PARSE_MATH_EXPONENT,
    STATE_DISABLED
};

enum ParamEnum
{
    PARAM_NUM,
    PARAM_GLOBAL,
    PARAM_SYSTEM,
    PARAM_TEMP,
    PARAM_GLOBAL_SUB,
    PARAM_SUBFINAL,
    PARAM_SUBARRAY,
    PARAM_SUBPROP,
    PARAM_SUBARRAY_NUM,
    PARAM_SUBPROP_NUM,
    PARAM_STRING
};

typedef struct MissileStruct
{
    MyDouble X;
    MyDouble Y;
    MyDouble OldX;
    MyDouble OldY;
    MyDouble Heading;
    MyDouble Energy;
    struct MissileStruct *Next;
} MissileStruct;

typedef struct FunctionStruct
{
	int CmdLen;
	char *Name;
	char *Cmds;
    int CmdPos;         //bad things happen if the same function is used across multiple events and one of them is waiting for motion/etc
} FunctionStruct;

typedef struct CPUStateStruct
{
	unsigned int Regs[8];
} CPUStateStruct;

typedef struct EventTriggerStruct
{
    MyDouble Bearing;
    MyDouble Heading;
    MyDouble Distance;
    MyDouble Energy;
} EventTriggerStruct;

//updates to this struct require updating states.S
typedef struct StateStruct
{
    CPUStateStruct CPU;
    unsigned int OrigPC;
    int CurState;
    int EventLevel;
    void *Stack;
    void *OrigStack;
    FunctionStruct *Function;	//function pointer for this event
    int TriggerCount;
    EventTriggerStruct Triggers[2]; //this is bad, really should be a linked list, would hate to have someone turn off the detections and let them accumulate :)
} StateStruct;

typedef struct GlobalVarStruct
{
    int ArraySize;
    int PropSize;
    MyDouble Value;
    char *Name;
    struct GlobalVarStruct *Array;
    struct GlobalVarStruct *Prop;
} GlobalVarStruct;

//anything causing StateStruct *Events to be moved requires an update to states.S
//careful crafting of _tempval_x values will allow rewriting the globalvars to point to events
//and a careful craft of it should allow rewriting stack, pc, etc of events along with reading them
typedef struct RobotStruct
{
    MyDouble TempVars[10];        //fixed size, probably bad..
    StateStruct *Events[MAX_EVENTS];
    char Name[32];                //BUG, we don't validate length in the parsing
    GlobalVarStruct *GlobalVars;
    int GlobalSize;
    MyDouble SystemVars[SYSTEM_VAR_COUNT];
    int Stalled;
    int RotationLock;	//1 = body lock, 2 = gun lock
    MyDouble OldX;
    MyDouble OldY;
    MyDouble OldEnergy;
    int Decelerating;
    int StopMotion;
    int StopRotation;
    int ScanActive;
    int AutoScan;
    int Blocking;
    int FunctionCount;
    FunctionStruct **Functions;
    int Initialized;
    int Alive;
} RobotStruct;

extern volatile struct MissileStruct *Missiles;
extern volatile struct StateStruct GlobalState;
extern volatile struct StateStruct *CurrentState;
extern volatile char *ErrorMsg;

RobotStruct *ParseRobot(char *RobotCmds);

void NEXT_STATE(enum StateEnum NewState) __attribute__((noreturn));
void SWAP_TO_STATE(StateStruct *State) __attribute__((noreturn));
void UPDATE_EVENT_STATE(StateStruct *State, int SwapState);
void SAVE_EVENT_STATE(StateStruct *State, int SwapState);
void SWAP_TO_EVENT(RobotStruct *RobotData, int Event) __attribute__((noreturn));
void CREATE_GLOBAL_STATE(StateStruct *State);
void FREE_STATE(StateStruct *State);

#endif
