#ifndef PARSECMDS_H
#define PARSECMDS_H

#include "cmds.h"

typedef int (*CmdParsePtr)(char **Params[], FunctionStruct *Func, RobotStruct *Robot);

typedef struct CommandParserStruct
{
    enum CommandEnum ID;
    char *Name;
    int ParamCount;
    CmdParsePtr Handler;
} CommandParserStruct;

enum BlockType
{
    BLOCK_IF,
    BLOCK_ELSE,
    BLOCK_WHILE
};

typedef struct BlockOffsetStruct
{
    unsigned int Offset;
    struct BlockOffsetStruct *Next;
} BlockOffsetStruct;

typedef struct BlockInfoStruct
{
    unsigned int Type;
    unsigned int Offset;
    struct BlockInfoStruct *ParentBlock;
    struct BlockOffsetStruct *ExtraOffsets;
} BlockInfoStruct;

extern CommandParserStruct ParseCmds[];
extern BlockInfoStruct *CurrentBlock;
extern int OffsetAtLineStart;

void EraseTempVars(int StartPos);
char *CreateTempVar(int *ID);

int cmdParseAssignMath(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseName(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseAhead(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseBack(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseAccelTarget(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseDecelTarget(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseSpeedTarget(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseBodyLeft(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseBodyRight(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseGunLeft(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseGunRight(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseRadarLeft(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseRadarRight(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseStop(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseStopMove(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseStopRotate(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseContinue(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseScan(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseFire(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseSyncAll(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseSyncGun(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseLockAll(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseLockGun(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseEndTurn(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseStall(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseSuicide(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseRegAScan(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseRegCldRobot(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseRegCldMissile(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseRegCldWall(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseRegCore(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseRegDtcRobot(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseRegDtcWall(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseRegPing(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseAScanEvents(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseCldWallEvents(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseCldRobotEvents(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseCoreEvents(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseDtcRobotEvents(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseDtcWallEvents(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParsePingEvents(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseCldMissileEvents(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseBlocking(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseWaitFor(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseGoSub(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseReturn(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseIf(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseElseIf(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseElse(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseEndIf(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseWhile(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseBreak(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseNext(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseEndWhile(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseAbs(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseMax(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseMin(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseRound(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseTruncate(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseGetTime(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseGetRandom(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParseIsEqual(char **Params[], FunctionStruct *Func, RobotStruct *Robot);
int cmdParsePrint(char **Params[], FunctionStruct *Func, RobotStruct *Robot);

#endif