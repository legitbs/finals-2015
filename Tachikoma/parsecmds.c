#include "robotbattle.h"
#include "parsecmds.h"
#include "cmds.h"
#include <stdlib.h>
#include <string.h>

BlockInfoStruct *CurrentBlock;
int OffsetAtLineStart;

CommandParserStruct ParseCmds[] = {
//    {0, "AssignMath", 0, 0},
    {CMD_Name, "name", -1, cmdParseName},
    {CMD_Ahead, "ahead", 1, cmdParseAhead},
    {CMD_Back, "back", 1, cmdParseBack},
    {CMD_AccelTarget, "acceltarget", 1, cmdParseAccelTarget},
    {CMD_DecelTarget, "deceltarget", 1, cmdParseDecelTarget},
    {CMD_SpeedTarget, "speedtarget", 1, cmdParseSpeedTarget},
    {CMD_BodyLeft, "bodyleft", 1, cmdParseBodyLeft},
    {CMD_BodyRight, "bodyright", 1, cmdParseBodyRight},
    {CMD_GunLeft, "gunleft", 1, cmdParseGunLeft},
    {CMD_GunRight, "gunright", 1, cmdParseGunRight},
    {CMD_RadarLeft, "radarleft", 1, cmdParseRadarLeft},
    {CMD_RadarRight, "radarright", 1, cmdParseRadarRight},
    {CMD_Stop, "stop", 0, cmdParseStop},
    {CMD_StopMove, "stopmove", 0, cmdParseStopMove},
    {CMD_StopRotate, "stoprotate", 0, cmdParseStopRotate},
    {CMD_Continue, "continue", 0, cmdParseContinue},
    {CMD_Scan, "scan", 0, cmdParseScan},
    {CMD_Fire, "fire", 1, cmdParseFire},
    {CMD_SyncAll, "syncall", 0, cmdParseSyncAll},
    {CMD_SyncGun, "syncgun", 0, cmdParseSyncGun},	//20
    {CMD_LockAll, "lockall", 1, cmdParseLockAll},
    {CMD_LockGun, "lockgun", 1, cmdParseLockGun},
    {CMD_EndTurn, "endturn", 0, cmdParseEndTurn},
    {CMD_Stall, "stall", 1, cmdParseStall},
    {CMD_Suicide, "suicide", 0, cmdParseSuicide},
    {CMD_RegAScan, "regascan", 2, cmdParseRegAScan},
    {CMD_RegCldRobot, "regcldrobot", 2, cmdParseRegCldRobot},
    {CMD_RegCldMissile, "regcldmissile", 2, cmdParseRegCldMissile},
    {CMD_RegCldWall, "regcldwall", 2, cmdParseRegCldWall},
    {CMD_RegCore, "regcore", 1, cmdParseRegCore},
    {CMD_RegDtcRobot, "regdtcrobot", 2, cmdParseRegDtcRobot},
    {CMD_RegDtcWall, "regdtcwall", 2, cmdParseRegDtcWall},
    {CMD_RegPing, "regping", 2, cmdParseRegPing},
    {CMD_AScanEvents, "ascanevents", 1, cmdParseAScanEvents},
    {CMD_CldWallEvents, "cldwallevents", 1, cmdParseCldWallEvents},
    {CMD_CldRobotEvents, "cldrobotevents", 1, cmdParseCldRobotEvents},
    {CMD_CoreEvents, "coreevents", 1, cmdParseCoreEvents},
    {CMD_DtcRobotEvents, "dtcrobotevents", 1, cmdParseDtcRobotEvents},
    {CMD_DtcWallEvents, "dtcwalevents", 1, cmdParseDtcWallEvents},
    {CMD_PingEvents, "pingevents", 1, cmdParsePingEvents},
    {CMD_CldMissileEvents, "cldmissileevents", 1, cmdParseCldMissileEvents},	//40
    {CMD_Blocking, "blocking", 1, cmdParseBlocking},
    {CMD_WaitFor, "waitfor", 1, cmdParseWaitFor},
//    {CMD_GoSub, "gosub", 1, cmdParseGoSub},
//    {CMD_Return, "return", 0, cmdParseReturn},
    {CMD_If, "if", 1, cmdParseIf},
    {CMD_ElseIf, "elseif", 1, cmdParseElseIf},
    {CMD_Else, "else", 0, cmdParseElse},
    {CMD_EndIf, "endif", 0, cmdParseEndIf},
    {CMD_While, "while", 1, cmdParseWhile},
    {CMD_Break, "break", 0, cmdParseBreak},
    {CMD_Next, "next", 0, cmdParseNext},
    {CMD_EndWhile, "endw", 0, cmdParseEndWhile},
    {CMD_Abs, "abs", 1, cmdParseAbs},
    {CMD_Max, "max", 2, cmdParseMax},
    {CMD_Min, "min", 2, cmdParseMin},
    {CMD_Round, "round", 2, cmdParseRound},
    {CMD_Truncate, "truncate", 1, cmdParseTruncate},
    {CMD_GetTime, "gettime", 0, cmdParseGetTime},
    {CMD_GetRandom, "getrandom", 0, cmdParseGetRandom},
    {CMD_IsEqual, "isequal", 2, cmdParseIsEqual},
    {CMD_Print, "print", 0xffffffff, cmdParsePrint},	//61
/*
    {0, "cmdAdd", 0, 0},
    {0, "cmdSub", 0, 0},
    {0, "cmdModulu", 0, 0},
    {0, "cmdMultiply", 0, 0},
    {0, "cmdDivide", 0, 0},
    {0, "cmdExponent", 0, 0},
    {0, "cmdAnd", 0, 0},
    {0, "cmdOr", 0, 0},
    {0, "cmdEquals", 0, 0},
    {0, "cmdLessThan", 0, 0},
    {0, "cmdLessThanEqualTo", 0, 0},
    {0, "cmdGreaterThan", 0, 0},
    {0, "cmdGreaterThanEqualTo", 0, 0},
    {0, "cmdNotEqual", 0, 0},
    {0, "cmdCos", 0, 0},
    {0, "cmdSin", 0, 0},
    {0, "cmdTan", 0, 0},
    {0, "cmdACos", 0, 0},
    {0, "cmdASin", 0, 0},
    {0, "cmdATan", 0, 0},
    {0, "cmdATan2", 0, 0},
    {0, "Jmp", 0, 0},
*/
    {-1, 0, -1, 0},
};

int CheckIsFloat(char *str)
{
    while(*str)
    {
        if(((*str < 0x30) || (*str > 0x39)) && (*str != '.') && (*str != '-'))
            return 0;
        
        str++;
    }
    
    return 1;
}

int IDAndOutputVariable(char **Param, FunctionStruct *Func, RobotStruct *Robot)
{
    MyDouble MyDoubleVal;
    int i;
    int BracketFlag = 0;
    char Error[400];
    char *SplitLines[3];
    int LineCount;
    int x, y;
    GlobalVarStruct *CurVar;
    GlobalVarStruct *TempVar;
    int NumParams;
    
    //this code adds a param to the function list
    //it expects that the incoming param is either the name of
    //a system or global entry
    //or a global entry that has been condensed on it's array portions
    //example a[ x + y ] should come in as a[ z ]
    //returns the number of fields it used
    
    if(Param[0] && CheckIsFloat(Param[0]))
    {
        //Param is a raw number, just store it as is
        MyDoubleVal = my_atof(Param[0]);
        Func->Cmds[Func->CmdLen] = PARAM_NUM;
        *(MyDouble *)&Func->Cmds[Func->CmdLen+1] = MyDoubleVal;
        Func->CmdLen += 1 + sizeof(struct MyDouble);
        return 1;
    }
    else
    {
        //need to go find it in the list
        LowerCase(Param[0]);
        if(Param[0][0] == '_')
        {
            //should be a system entry
            i = GetSystemVarID(Param[0]);
            if(i != -1)
            {
                //found the global entry
                Func->Cmds[Func->CmdLen] = PARAM_SYSTEM;
                Func->Cmds[Func->CmdLen+1] = i;
                Func->CmdLen += 2;
                return 1;
            }
            
            //couldn't find it, check the temporary variable name
            if(memcmp(Param[0], "_tresult_", 9) == 0)
            {
                //suppose to be internal created so just go grab the next param and use it
                //as is, could be a bad thing
                i = my_atoi(&Param[0][9]);
                Func->Cmds[Func->CmdLen] = PARAM_TEMP;
                Func->Cmds[Func->CmdLen+1] = i;
                Func->CmdLen += 2;
                return 1;
            }
            
            //user vars cant start with _ so it is bad
            i = strlen(Param[0]) + 40;
            ErrorMsg = Error;

            //BUG - big enough param might actually cause trouble depending on memory layout
            my_snprintf(ErrorMsg, i, "Unable to locate system variable %s", Param[0]);
            NEXT_STATE(PARSE_ERROR);
        }
        
        //not a system variable, scan user variables for it
        NumParams = 1;
        i = my_strlen(Param[0]);
        if((Param[0][i-1] == '[') || (Param[0][i-1] == 0x01))
        {
            BracketFlag = 1;
            Param[0][i-1] = 0;
        }
        
        //split on the . if it exists
        LineCount = SplitData(Param[0], (char **)&SplitLines, '.', 0, 3);
        
        for(i = 0; i < Robot->GlobalSize; i++)
        {
            if(my_strcmp(Robot->GlobalVars[i].Name, SplitLines[0]) == 0)
                break;
        }

        //if we couldn't find it, add it
        if(i == Robot->GlobalSize)
        {
            //allocate a new entry
            TempVar = malloc(sizeof(struct GlobalVarStruct)*(i+1));
            memcpy(TempVar, Robot->GlobalVars, sizeof(struct GlobalVarStruct)*i);
            memset(&TempVar[i], 0, sizeof(struct GlobalVarStruct));
            
            //setup the name
            x = my_strlen(SplitLines[0]);
            if(x > 30)
            {
                ErrorMsg = "Variable name too long";
                NEXT_STATE(PARSE_ERROR);
            }
            TempVar[i].Name = malloc(x+1);
            memcpy(TempVar[i].Name, SplitLines[0], x);
            TempVar[i].Name[x] = 0;
            
            //free and reset the global list
            if(Robot->GlobalVars)
                free(Robot->GlobalVars);
            Robot->GlobalVars = TempVar;
            Robot->GlobalSize++;
        }
        
        //see if we have sub entries
        if(!((LineCount >= 2) || BracketFlag))
        {
            //no bracket and no property, just indicate global
            Func->Cmds[Func->CmdLen] = PARAM_GLOBAL;
            Func->Cmds[Func->CmdLen+1] = i;
            Func->CmdLen += 2;
        }
        else
        {
            //indicate we have sub entries
            Func->Cmds[Func->CmdLen] = PARAM_GLOBAL_SUB;
            Func->Cmds[Func->CmdLen+1] = i;
            Func->CmdLen += 2;
            
            CurVar = &Robot->GlobalVars[i];
            while(1)
            {
                //if we have line count then handle the property
                if(LineCount >= 2)
                {
                    LowerCase(SplitLines[1]);
                    if(CurVar)
                    {
                        if(my_strlen(SplitLines[1]) == 0)
                        {
                            ErrorMsg = "Missing property name";
                            NEXT_STATE(PARSE_ERROR);
                        }
                        
                        for(x = 0; x < CurVar->PropSize; x++)
                        {
                            if(my_strcmp(SplitLines[1], CurVar->Prop[x].Name) == 0)
                                break;
                        }
                        if(x >= CurVar->PropSize)
                        {
                            //can't find this property, add it
                            TempVar = malloc(sizeof(struct GlobalVarStruct)*(CurVar->PropSize+1));
                            
                            //copy the old block into the new one and free the old one
                            memcpy(TempVar, CurVar->Prop, sizeof(struct GlobalVarStruct)*CurVar->PropSize);
                            CurVar->PropSize++;
                            if(CurVar->Prop)
                                free(CurVar->Prop);
                            
                            //setup the new entry
                            CurVar->Prop = TempVar;
                            TempVar = &TempVar[CurVar->PropSize];
                            memset(TempVar, 0, sizeof(struct GlobalVarStruct));
                            
                            //copy the name in
                            y = my_strlen(SplitLines[1]);
                            if(y > 30)
                            {
                                ErrorMsg = "Property name too long";
                                NEXT_STATE(PARSE_ERROR);
                            }
                            
                            TempVar->Name = malloc(y + 1);
                            memcpy(TempVar->Name, SplitLines[1], y);
                            TempVar->Name[y] = 0;
                        }
                        
                        //now add which id this property is
                        Func->Cmds[Func->CmdLen] = PARAM_SUBPROP_NUM;
                        Func->Cmds[Func->CmdLen+1] = x;
                        Func->CmdLen += 2;
                    }
                    else
                    {
                        //we don't know if the property exists so add in our special entry
                        //as it has to be looked up each time
                        Func->Cmds[Func->CmdLen] = PARAM_SUBPROP;
                        y = my_strlen(SplitLines[1]);
                        if(y > 30)
                        {
                            ErrorMsg = "Property name too long";
                            NEXT_STATE(PARSE_ERROR);
                        }
                        
                        Func->Cmds[Func->CmdLen+1] = y;
                        memcpy(&Func->Cmds[Func->CmdLen+2], SplitLines[1], y);
                        Func->CmdLen += 2 + y;
                    }
                
                    //if the line split is more than 2 entries then we need to shift and continue parsing
                    //more properties
                    if(LineCount > 2)
                    {
                        //another property exists, shift Param[0] over and resplit
                        if(CurVar)
                            CurVar = &CurVar->Prop[x];
                        Param[0] = SplitLines[1];
                        Param[0][my_strlen(Param[0])] = '.';
                        LineCount = SplitData(Param[0], (char **)&SplitLines, '.', 0, 3);
                        continue;
                    }
                }
                
                //if we have a bracket, we need to dig deeper
                //we should have at least 2 more entries after us in that case
                //due to a[ x ], a[ x ].prop, or a[ x ][ y ]...
                //when we are called it is expected that any value being used
                //for x or y is not an array itself
                if(BracketFlag)
                {
                    if((Param[1] == 0) || (Param[2] == 0))
                    {
                        ErrorMsg = "Global variable syntax error";
                        NEXT_STATE(PARSE_ERROR);
                    }
                }
                else
                {
                    //must be just the property entry
                    Func->Cmds[Func->CmdLen] = PARAM_SUBFINAL;
                    Func->CmdLen++;
                    return NumParams;
                }
                
                //we have a bracket to get this far, indicate as such and the id for it
                if(my_isnumeric(Param[1]))
                {
                    Func->Cmds[Func->CmdLen] = PARAM_SUBARRAY_NUM;
                    x = my_atoi(Param[1]);
                    *(int *)&Func->Cmds[Func->CmdLen+1] = x;
                    Func->CmdLen += 1 + sizeof(int);
                    NumParams++;
                    
                    if(CurVar)
                    {
                        if(x >= CurVar->ArraySize)
                        {
                            //resize our array
                            //can't find this property, add it
                            TempVar = malloc(sizeof(struct GlobalVarStruct)*(x+1));
                            
                            //copy the old block into the new one and free the old one
                            memcpy(TempVar, CurVar->Array, sizeof(struct GlobalVarStruct)*CurVar->ArraySize);
                            memset(&TempVar[CurVar->ArraySize], 0, sizeof(struct GlobalVarStruct)*((x+1)-CurVar->ArraySize));
                            CurVar->ArraySize = x+1;
                            
                            if(CurVar->Array)
                                free(CurVar->Array);
                            
                            //setup the new entry
                            CurVar->Array = TempVar;
                            TempVar = &TempVar[x-1];
                        }
                        CurVar = &CurVar->Array[x];
                    }
                }
                else
                {
                    //make sure we stop looking things up
                    CurVar = 0;
                    Func->Cmds[Func->CmdLen] = PARAM_SUBARRAY;
                    Func->CmdLen++;
                    
                    //call us and let it output details of the sub entry
                    NumParams += IDAndOutputVariable(&Param[1], Func, Robot);
                }

                //the first character should be a ] due to being an array
                if((Param[2][0] != ']') && (Param[2][0] != 0x02))
                {
                    ErrorMsg = "Missing ]";
                    NEXT_STATE(PARSE_ERROR);
                }

                //see if there are any entries after us
                if(Param[2][1] == 0)
                {
                    //no more entries to track, we are on the last entry
                    Func->Cmds[Func->CmdLen] = PARAM_SUBFINAL;
                    Func->CmdLen++;
                    return NumParams+1;
                }
                else
                {
                    //see if it is another array entry or a property
                    NumParams++;
                    if(Param[2][1] == '.')
                    {
                        //a[ x ].prop setup
                        //now figure out the property
                        
                        //advance the param ahead
                        Param += 2;
                        
                        //if it ends with a [ then setup the values properly
                        BracketFlag = 0;
                        x = my_strlen(Param[0]);
                        if((Param[0][x-1] == '[') || (Param[0][x-1] == 0x01))
                        {
                            BracketFlag = 1;
                            Param[0][x-1] = 0;
                        }
                        
                        LineCount = SplitData(Param[0], (char **)&SplitLines, '.', 0, 3);
                        
                        //techncially I should be looking for ].[ or similar things as being invalid *shrugs*
                        
                        //loop back around
                        continue;
                    }
                    else if(((Param[2][1] == '[') || (Param[2][1] == 0x01)) && (Param[2][2] == 0))
                    {
                        //a[ x ][ y ]
                        
                        //setup for the [ y ] portion and loop
                        BracketFlag = 1;
                        Param += 2;
                        LineCount = 0;
                        continue;
                    }
                    else
                    {
                        ErrorMsg = "Param bracket parsing error";
                        NEXT_STATE(PARSE_ERROR);
                    }
                }
            }
        }
    }
    return NumParams;
}

struct MathOrderStruct
{
    char *MathOp;
    int MathID;
} MathOrderStruct;

struct MathOrderStruct MathOrder8[] =
{
    {"||", CMD_Or},
    {0, 0}
};

struct MathOrderStruct MathOrder7[] =
{
    {"&&", CMD_And},
    {0, 0}
};

struct MathOrderStruct MathOrder6[] =
{
    {"!=", CMD_NotEqual},
    {"<>", CMD_NotEqual},
    {"==", CMD_Equals},
    {0, 0}
};

struct MathOrderStruct MathOrder5[] =
{
    {">=", CMD_GreaterThanEqualTo},
    {">", CMD_GreaterThan},
    {"<=", CMD_LessThanEqualTo},
    {"<", CMD_LessThan},
    {0, 0}
};

struct MathOrderStruct MathOrder4[] =
{
    {"-", CMD_Sub},
    {"+", CMD_Add},
    {0, 0}
};

struct MathOrderStruct MathOrder3[] =
{
    {"%", CMD_Modulus},
    {"/", CMD_Divide},
    {"*", CMD_Multiply},
    {0, 0}
};

struct MathOrderStruct MathOrder2[] =
{
    {"^", CMD_Exponent},
    {0, 0}
};

//ID number of where atan2() is in the below list
#define ATAN2_MATHOP 6
struct MathOrderStruct MathOrder1[] =
{
    {"sin", CMD_Sin},
    {"cos", CMD_Cos},
    {"tan", CMD_Tan},
    {"asin", CMD_ASin},
    {"acos", CMD_ACos},
    {"atan", CMD_ATan},
    {"atan2", CMD_ATan2},
    {0, 0}
};

//order of operations so we evaluate properly
struct MathOrderStruct *MathOrder[] =
{
    MathOrder2, MathOrder3, MathOrder4,
    MathOrder5, MathOrder6, MathOrder7, MathOrder8,
    0
};

typedef struct TempVarStruct
{
    char *Name;
    struct TempVarStruct *Next;
} TempVarStruct;
TempVarStruct *TempHead = 0;

void EraseTempVars(int StartPos)
{
    TempVarStruct *CurTemp;
    TempVarStruct *NextTemp;
    TempVarStruct *StartTemp;
    int i;
    
    //erase all entries that are above StartPos
    CurTemp = TempHead;
    StartTemp = 0;
    for(i = 0; i < StartPos; i++)
    {
        StartTemp = CurTemp;
        CurTemp = CurTemp->Next;
    }
    
    //now free everything
    while(CurTemp)
    {
        NextTemp = CurTemp;
        CurTemp = CurTemp->Next;
        free(NextTemp->Name);
        free(NextTemp);
    }
    
    //if no entries saved then wipe out the head
    if(StartPos == 0)
        TempHead = 0;
    else
        StartTemp->Next = 0;
}

char *CreateTempVar(int *ID)
{
    int TempID;
    int NameLen;
    TempVarStruct *CurEntry;
    TempVarStruct *NewEntry;

    //if no ID area provided then use our temp
    if(ID == 0)
        ID = &TempID;
    
    //output the temporary variable and increment to the next one
    CurEntry = TempHead;
    *ID = 0;
    if(CurEntry)
    {
        (*ID)++;
        while(CurEntry->Next)
        {
            CurEntry = CurEntry->Next;
            (*ID)++;
        };
    }
    
    //create the new entry
    NewEntry = malloc(sizeof(TempVarStruct));
    NewEntry->Next = 0;
    NameLen = 9 + my_numlen(*ID);
    NewEntry->Name = malloc(NameLen + 1);
    my_snprintf(NewEntry->Name, NameLen + 1, "_tresult_%d", *ID);
    NewEntry->Name[NameLen] = 0;
    
    //if we have an entry then add to it
    if(CurEntry)
        CurEntry->Next = NewEntry;
    else
        TempHead = NewEntry;
    
    return NewEntry->Name;
}

char *OutputTempVar(FunctionStruct *Func)
{
    //now add it to the function list
    int ID;
    char *Name;
    
    Name = CreateTempVar(&ID);
    Func->Cmds[Func->CmdLen] = PARAM_TEMP;
    Func->Cmds[Func->CmdLen+1] = ID;
    Func->CmdLen+=2;

    return Name;
}

//we need to take a list of math input and rearrange it in an order useful to us
//so it equates properly
//find inner most parens and process it
//then areas inside of brackets for inputs to arrays
//then handle order of math operations
//this function should be recursive when processing, could be bad, not sure if it is abuseable
//it will alter the params. any data removed is assigned to a temp value except when an = is found
int cmdParseHandleMath(char **Param, int ParamCount, FunctionStruct *Func, RobotStruct *Robot)
{
    //scan for inner most ()
    int i;
    int InnerMost;
    int CurMathOrder;
    int CurSubMathOrder;
    int LastMath;
    int TempMathOrder;
    int TempSubMathOrder;
    int DidMathParse;
    
    DidMathParse = 0;
    while(1)
    {
        InnerMost = -1;
        LastMath = -1;
        for(i = 0; i < ParamCount; i++)
        {
            if(Param[i][0] == '(')
            {
                if(Param[i][1] != 0)
                {
                    ErrorMsg = "Extra data after (";
                    NEXT_STATE(PARSE_ERROR);
                }
                InnerMost = i;
                LastMath = -1;
            }
            else if(Param[i][strlen(Param[i]) - 1] == '(')
            {
                //ends with (, see if it matches sin(), etc
                Param[i][strlen(Param[i]) - 1] = 0;
                for(CurMathOrder = 0; MathOrder1[CurMathOrder].MathOp; CurMathOrder++)
                {
                    if(my_strcmp(Param[i], MathOrder1[CurMathOrder].MathOp) == 0)
                    {
                        //use LastMath as a flag to indicate we have a math operator to handle
                        InnerMost = i;
                        LastMath = CurMathOrder;
                        Param[i][strlen(Param[i]) - 1] = '(';
                        break;
                    }
                }
                
                //if we hit the end then fail it
                if(MathOrder1[CurMathOrder].MathOp == 0)
                {
                    ErrorMsg = "Unrecognized command before (";
                    NEXT_STATE(PARSE_ERROR);
                }
            }
        }

        if(InnerMost != -1)
        {
            //we found the inner most ( or worst case a () at the end after a logic operator
            //find the end after it then call ourselves with the details
            //we expect that upon return that Param[] has been modified to remove the () along with
            //everything inside of it. The resulting entry should be a temporary variable holding the result of any checks or math
            for(i = InnerMost + 1; i < ParamCount; i++)
            {
                if(Param[i][0] == ')')
                {
                    if(Param[i][1] != 0)
                    {
                        ErrorMsg = "Extra data after )";
                        NEXT_STATE(PARSE_ERROR);
                    }
                    break;
                }
            }
            
            if(i >= ParamCount)
            {
                ErrorMsg = "Unable to locate closing )";
                NEXT_STATE(PARSE_ERROR);
            }

            //handle the sub area if there is too many params
            if(i - InnerMost - 1 > 1)
            {
                //see if this is tan2(,) and handle the comma if need be
                if(LastMath == ATAN2_MATHOP)
                {
                    for(CurMathOrder = InnerMost; CurMathOrder < i; CurMathOrder++)
                    {
                       if(Param[CurMathOrder][strlen(Param[CurMathOrder]) - 1] == ',')
                            break;
                    }
                    
                    //if too far then we couldn't find the comma
                    if(CurMathOrder >= i)
                    {
                        //can't find the comma
                        ErrorMsg = "Unable to locate , for atan2";
                        NEXT_STATE(PARSE_ERROR);
                    }

                    //remove the comma, if more than 1 entry then call ourselves
                    Param[CurMathOrder][strlen(Param[CurMathOrder]) - 1] = 0;
                    if(CurMathOrder - InnerMost - 1 > 1)
                    {
                        cmdParseHandleMath(&Param[InnerMost + 1], CurMathOrder - InnerMost - 1, Func, Robot);
                        DidMathParse = 1;
                    }
                    
                    //math parse the second param if need be
                    if(i - CurMathOrder - 1 > 1)
                    {
                        cmdParseHandleMath(&Param[CurMathOrder + 1], i - CurMathOrder - 1, Func, Robot);
                        DidMathParse = 1;
                    }
                }
                else
                {
                    cmdParseHandleMath(&Param[InnerMost + 1], i - InnerMost - 1, Func, Robot);
                    DidMathParse = 1;
                }
            }

            //if we have a math function then let it process
            if(LastMath != -1)
            {
                //we've handled params to the math function, call it at this point and store off to a temp area
                Func->Cmds[Func->CmdLen] = MathOrder1[LastMath].MathID;
                Func->CmdLen++;
                TempMathOrder = IDAndOutputVariable(&Param[InnerMost + 1], Func, Robot);
                TempSubMathOrder = 0;

                if(LastMath == ATAN2_MATHOP)
                {
                    //add the extra param for ATAN2
                    TempSubMathOrder = IDAndOutputVariable(&Param[CurMathOrder + 1], Func, Robot);
                }
                    
                //assign to InnerMost+1 the output variable used
                Param[InnerMost+1] = OutputTempVar(Func);
            }

            //shift all of the other pointers down
            Param[InnerMost] = Param[InnerMost+1];
            memcpy(&Param[InnerMost+1], &Param[i + 1], sizeof(char *)*(ParamCount - i - 1));
            
            //Param should be modified now and only be 1 in size, shrink everything
            ParamCount -= (i - InnerMost);
        }
        else
            break;  //no more () to collapse
    }
    
    //now collapse all [] due to the arrays needing a final variable to work with
    while(1)
    {
        InnerMost = -1;
        for(i = 0; i < ParamCount; i++)
        {
            if(Param[i][strlen(Param[i]) - 1] == '[')
                InnerMost = i;
        }
        
        if(InnerMost != -1)
        {
            //find the closing ], we don't look for data after as it may be part of a property
            for(i = InnerMost + 1; i < ParamCount; i++)
            {
                if(Param[i][0] == ']')
                    break;
            }
            
            if(i >= ParamCount)
            {
                ErrorMsg = "Unable to locate closing ]";
                NEXT_STATE(PARSE_ERROR);
            }
            
            //swap out the [] with char(1) and char(2) to avoid this loop getting stuck and to show
            //we already handled the entry
            Param[InnerMost][strlen(Param[InnerMost]) - 1] = 0x01;
            Param[i][0] = 0x02;

            //handle the sub area if there is more than 1 property
            if((i - InnerMost - 1) > 1)
            {
                cmdParseHandleMath(&Param[InnerMost + 1], i - InnerMost - 1, Func, Robot);

                //shift all of the other pointers down
                memcpy(&Param[InnerMost+2], &Param[i], sizeof(char *)*(ParamCount - i));

                //Param should be modified, shrink the count we have
                ParamCount -= (i - InnerMost - 2);
            }
        }
        else
            break;  //no more [] to collapse the inside of
    }
    
    //now we should just have raw math left
    //do the math as required based on the param type
    //keep in mind that user types may have arrays and need extra handling

    //keep order of operations, see if anything exists in the entry, if so then collapse
    CurMathOrder = 0;
    while(MathOrder[CurMathOrder])
    {
        InnerMost = -1;
        for(i = 0; i < ParamCount; i++)
        {
            for(CurSubMathOrder = 0; MathOrder[CurMathOrder][CurSubMathOrder].MathOp; CurSubMathOrder++)
            {
                if(my_strcmp(Param[i], MathOrder[CurMathOrder][CurSubMathOrder].MathOp) == 0)
                {
                    InnerMost = i;
                    break;
                }
            }
            
            //if we found a match then use it
            if(InnerMost != -1)
                break;
        }
        
        //if we couldn't find anything then continue into the next list of params
        if(InnerMost == -1)
        {
            CurMathOrder++;
            continue;
        }
        
        //found an entry, grab each side then remove the entries from the list
        //we need to know where the previous math operation was
        //as we might be dealing with a user array that could take up
        //multiple slots
        LastMath = -1;
        for(i = InnerMost - 1; i >= 0; i--)
        {
            //we only need to look for current and future math as anything we've done so far has to be a higher priority
            //and would already be removed
            for(TempMathOrder = CurMathOrder; MathOrder[TempMathOrder]; TempMathOrder++)
            {
                for(TempSubMathOrder = 0; MathOrder[TempMathOrder][TempSubMathOrder].MathOp; TempSubMathOrder++)
                {
                    if(my_strcmp(Param[i], MathOrder[TempMathOrder][TempSubMathOrder].MathOp) == 0)
                    {
                        LastMath = i;
                        break;
                    }
                }
                
                //if we found a math option then stop looking
                if(LastMath != -1)
                    break;
            }
        };

        //setup LastMath an entry ahead of the operator
        LastMath++;

        //output the function we'll be doing and collapse
        DidMathParse = 1;
        Func->Cmds[Func->CmdLen] = MathOrder[CurMathOrder][CurSubMathOrder].MathID;
        Func->CmdLen++;
        TempMathOrder = IDAndOutputVariable(&Param[LastMath], Func, Robot);
        TempSubMathOrder = IDAndOutputVariable(&Param[InnerMost + 1], Func, Robot);
        
        //assign to LastMath the output variable used
        Param[LastMath] = OutputTempVar(Func);
        
        //TempMathOrder and TempSubMathOrder are the number of entries removed
        //adjust the params accordingly as we should just have 1 entry now for this math
        //Param[LastMath] contains the pointer to the new temp variable
  
        //copy anything after the math to be at the entry after our temp variable
        memcpy(&Param[LastMath+1], &Param[InnerMost+1+TempSubMathOrder], sizeof(char *) * (ParamCount - InnerMost - TempSubMathOrder - 1));
        
        //remove the math operation along with the two variables used
        ParamCount -= (TempMathOrder + TempSubMathOrder);

        //nothing left to collapse then exit the function
        if(ParamCount == 1)
            return 0;
        
        //if somehow we don't have enough params then abort
        if(ParamCount < 1)
        {
            ErrorMsg = "Error in parsing math operations";
            NEXT_STATE(PARSE_ERROR);
        }
    }
    
    //return if we did any math parsing at this level, we don't care about sub levels
    return DidMathParse;
}

int cmdParseAddFuncID(char **Param, FunctionStruct *Func, RobotStruct *Robot)
{
    int CurFunc = 0;
    
    if(Param[0] && (Param[1] == 0))
    {
        //param should be a string, we need to find a matching function name
        LowerCase(Param[0]);
        if(Param[0][0] != '"')
        {
            ErrorMsg = "Missing quote";
            NEXT_STATE(PARSE_ERROR);
        }
        
        //remove the quotes
        Param[0]++;
        Param[0][my_strlen(Param[0]) - 1] = 0;
        
        while(Robot->Functions[CurFunc])
        {
        	if(my_strcmp(Robot->Functions[CurFunc]->Name, Param[0]) == 0)
        	{
        		//found our function
        		Func->Cmds[Func->CmdLen] = (unsigned char)CurFunc;
                Func->CmdLen++;
        		return 0;
        	}
        	CurFunc++;
        }
		
        CurFunc = my_strlen(Param[0]) + 30;
        ErrorMsg = malloc(CurFunc);
		
        my_snprintf(ErrorMsg, CurFunc, "Unable to locate function %s", Param[0]);
        NEXT_STATE(PARSE_ERROR);
    }

    ErrorMsg = "How did you get here?";
    NEXT_STATE(PARSE_ERROR);
    
    return 0;
}

int cmdParseAssignMath(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
	//a = b setup

    //this could be setup to do a pushstate/popstate type setup
    //so next_state calls go here and avoid loops
    int i, equalSign;
    
    for(i = 0, equalSign = 0; Params[i]; i++)
    {
        //even after finding it we continue as we are assigning to i the number of entries
        if((((char *)Params[i])[0] == '=') && (equalSign == 0))
            equalSign = i;
        if(((char *)Params[i])[0] == '"')
        {
            ErrorMsg = "Invalid quote";
            NEXT_STATE(PARSE_ERROR);
        }
    }
    
    if(i <= 2)
    {
        ErrorMsg = "Error in assigning values";
        NEXT_STATE(PARSE_ERROR);
    }

    if((equalSign == 0) || (equalSign >= (i-1)))
    {
        ErrorMsg ="Error locating equal sign";
        NEXT_STATE(PARSE_ERROR);
    }
    
    //parse up all of the math functionality
    cmdParseHandleMath(&Params[equalSign+1], i - equalSign - 1, Func, Robot);

    //make sure there is no math being done on the assignment part other than inside of brackets
    if(cmdParseHandleMath(&Params[0], equalSign, Func,Robot))
    {
        ErrorMsg = "Invalid math during assignment";
        NEXT_STATE(PARSE_ERROR);
    }
    
    //now Params[equalSign+1] should be a variable we can use so output the equal function
    Func->Cmds[Func->CmdLen] = CMD_AssignMath;
    Func->CmdLen++;
    IDAndOutputVariable(&Params[equalSign+1], Func, Robot);
    IDAndOutputVariable(&Params[0], Func, Robot);
	return 0;
}

int cmdParseAddParam(char **Params, FunctionStruct *Func, RobotStruct *Robot)
{
    //we fully expect that we have just 1 parameter which may or may not be a number, a temporary variable, or a known name
    char *TempParams[2];
    
    TempParams[0] = Params[0];
    TempParams[1] = 0;
    IDAndOutputVariable(&TempParams, Func, Robot);
    return 0;
}

int cmdParseName(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    int Len;

    //we can only assign name on init
    if(my_strcmp(Func->Name, "init") != 0)
    {
        ErrorMsg = "Can only assign name on init";
        NEXT_STATE(PARSE_ERROR);
    }
    
    //we won't actually add it to the commands, we just add it to the robot

    Len = strlen(Params[0][0]);
    if((Params[0][0][0] != '"') || (Params[0][0][Len-1] != '"'))
    {
        ErrorMsg = "Missing quote";
        NEXT_STATE(PARSE_ERROR);
    }
    
    //adjust the pointer
    Params[0][0]++;
    Len -= 2;

    //robots starting with @ aren't allowed as they are auto assigned names to avoid confusion
    if(Params[0][0][0] == '@')
    {
        ErrorMsg = "Assigned names cannot start with @";
        NEXT_STATE(PARSE_ERROR);
    }

    //BUG - we don't length check when copied in
    //copy the name and null terminate
    memcpy(Robot->Name, Params[0][0], Len);
    Robot->Name[Len] = 0;
    
	return 0;
}

int cmdParseAhead(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_Ahead;
    Func->CmdLen++;
    
    //Ahead has 1 math param, the parser should have already condensed it
    cmdParseAddParam(Params[0], Func, Robot);
	return 0;
}

int cmdParseBack(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_Back;
    Func->CmdLen++;
    
    cmdParseAddParam(Params[0], Func, Robot);
    return 0;
}

int cmdParseAccelTarget(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_AccelTarget;
    Func->CmdLen++;
    
    cmdParseAddParam(Params[0], Func, Robot);
	return 0;
}

int cmdParseDecelTarget(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_DecelTarget;
    Func->CmdLen++;
    
    cmdParseAddParam(Params[0], Func, Robot);
	return 0;
}

int cmdParseSpeedTarget(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_SpeedTarget;
    Func->CmdLen++;
    
    cmdParseAddParam(Params[0], Func, Robot);
	return 0;
}

int cmdParseBodyLeft(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_BodyLeft;
    Func->CmdLen++;
    
    cmdParseAddParam(Params[0], Func, Robot);
	return 0;
}

int cmdParseBodyRight(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_BodyRight;
    Func->CmdLen++;
    
    cmdParseAddParam(Params[0], Func, Robot);
	return 0;
}

int cmdParseGunLeft(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_GunLeft;
    Func->CmdLen++;
    
    cmdParseAddParam(Params[0], Func, Robot);
	return 0;
}

int cmdParseGunRight(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_GunRight;
    Func->CmdLen++;
    
    cmdParseAddParam(Params[0], Func, Robot);
	return 0;
}

int cmdParseRadarLeft(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_RadarLeft;
    Func->CmdLen++;
    
    cmdParseAddParam(Params[0], Func, Robot);
	return 0;
}

int cmdParseRadarRight(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_RadarRight;
    Func->CmdLen++;
    
    cmdParseAddParam(Params[0], Func, Robot);
	return 0;
}

int cmdParseStop(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_Stop;
    Func->CmdLen++;
	return 0;
}

int cmdParseStopMove(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_StopMove;
    Func->CmdLen++;
	return 0;
}

int cmdParseStopRotate(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_StopRotate;
    Func->CmdLen++;
	return 0;
}

int cmdParseContinue(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_Continue;
    Func->CmdLen++;
    return 0;
}

int cmdParseScan(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_Scan;
    Func->CmdLen++;
	return 0;
}

int cmdParseFire(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_Fire;
    Func->CmdLen++;	
    cmdParseAddParam(Params[0], Func, Robot);
    return 0;
}

int cmdParseSyncAll(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_SyncAll;
    Func->CmdLen++;
	return 0;
}

int cmdParseSyncGun(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_SyncGun;
    Func->CmdLen++;
	return 0;
}

int cmdParseLockAll(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_LockAll;
    Func->CmdLen++;
    
    cmdParseAddParam(Params[0], Func, Robot);
	return 0;
}

int cmdParseLockGun(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_LockGun;
    Func->CmdLen++;
    
    cmdParseAddParam(Params[0], Func, Robot);
	return 0;
}

int cmdParseEndTurn(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_EndTurn;
    Func->CmdLen++;
	return 0;
}

int cmdParseStall(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_Stall;
    Func->CmdLen++;
    
    cmdParseAddParam(Params[0], Func, Robot);
	return 0;
}

int cmdParseSuicide(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_Suicide;
    Func->CmdLen++;
	return 0;
}

int cmdParseRegAScan(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_RegAScan;
    Func->CmdLen++;
    
    cmdParseAddFuncID(Params[0], Func, Robot);
    cmdParseAddParam(Params[1], Func, Robot);
    return 0;
}

int cmdParseRegCldRobot(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_RegCldRobot;
    Func->CmdLen++;
    
    cmdParseAddFuncID(Params[0], Func, Robot);
    cmdParseAddParam(Params[1], Func, Robot);
	return 0;
}

int cmdParseRegCldMissile(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_RegCldMissile;
    Func->CmdLen++;
    
    cmdParseAddFuncID(Params[0], Func, Robot);
    cmdParseAddParam(Params[1], Func, Robot);
	return 0;
}

int cmdParseRegCldWall(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_RegCldWall;
    Func->CmdLen++;
    
    cmdParseAddFuncID(Params[0], Func, Robot);
    cmdParseAddParam(Params[1], Func, Robot);
	return 0;
}

int cmdParseRegCore(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_RegCore;
    Func->CmdLen++;
    
    cmdParseAddFuncID(Params[0], Func, Robot);
	return 0;
}

int cmdParseRegDtcRobot(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_RegDtcRobot;
    Func->CmdLen++;
    
    cmdParseAddFuncID(Params[0], Func, Robot);
    cmdParseAddParam(Params[1], Func, Robot);
	return 0;
}

int cmdParseRegDtcWall(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_RegDtcWall;
    Func->CmdLen++;
    
    cmdParseAddFuncID(Params[0], Func, Robot);
    cmdParseAddParam(Params[1], Func, Robot);
	return 0;
}

int cmdParseRegPing(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_RegPing;
    Func->CmdLen++;
    
    cmdParseAddFuncID(Params[0], Func, Robot);
    cmdParseAddParam(Params[1], Func, Robot);
	return 0;
}

int cmdParseAScanEvents(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_AScanEvents;
    Func->CmdLen++;
    
    cmdParseAddParam(Params[0], Func, Robot);
	return 0;
}

int cmdParseCldWallEvents(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_CldWallEvents;
    Func->CmdLen++;
    
    cmdParseAddParam(Params[0], Func, Robot);
	return 0;
}

int cmdParseCldRobotEvents(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_CldRobotEvents;
    Func->CmdLen++;
    
    cmdParseAddParam(Params[0], Func, Robot);
	return 0;
}

int cmdParseCoreEvents(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_CoreEvents;
    Func->CmdLen++;
    
    cmdParseAddParam(Params[0], Func, Robot);
	return 0;
}

int cmdParseDtcRobotEvents(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_DtcRobotEvents;
    Func->CmdLen++;
    
    cmdParseAddParam(Params[0], Func, Robot);
	return 0;
}

int cmdParseDtcWallEvents(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_DtcWallEvents;
    Func->CmdLen++;
    
    cmdParseAddParam(Params[0], Func, Robot);
    return 0;
}

int cmdParsePingEvents(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_PingEvents;
    Func->CmdLen++;
    
    cmdParseAddParam(Params[0], Func, Robot);
	return 0;
}

int cmdParseCldMissileEvents(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_CldMissileEvents;
    Func->CmdLen++;
    
    cmdParseAddParam(Params[0], Func, Robot);
	return 0;
}

int cmdParseBlocking(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_Blocking;
    Func->CmdLen++;
    
    cmdParseAddParam(Params[0], Func, Robot);
	return 0;
}

int cmdParseWaitFor(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_WaitFor;
    Func->CmdLen++;
    
    //add in the offset to jump to if the value isn't what we need
    *(short *)&Func->Cmds[Func->CmdLen] = OffsetAtLineStart - Func->CmdLen;
    Func->CmdLen += 2;
    
    cmdParseAddParam(Params[0], Func, Robot);
	return 0;
}

int cmdParseGoSub(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    /*
    Func->Cmds[Func->CmdLen] = CMD_GoSub;
    Func->CmdLen++;
    
    cmdParseAddFuncID(Params[0], Func, Robot);
     */
	return 0;
}

int cmdParseReturn(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    /*
    Func->Cmds[Func->CmdLen] = CMD_Return;
    Func->CmdLen++;
     */
	return 0;
}

int cmdParseIf(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    //create a new block entry and add it
    BlockInfoStruct *NewBlock = malloc(sizeof(BlockInfoStruct));
    NewBlock->Type = BLOCK_IF;
    NewBlock->ParentBlock = CurrentBlock;
    NewBlock->ExtraOffsets = 0;
    NewBlock->Offset = Func->CmdLen + 1;
    CurrentBlock = NewBlock;
    
    //output if command
    Func->Cmds[Func->CmdLen] = CMD_If;
    Func->CmdLen += 3; //advance the pointer 2 spots for the offset
    
    //add in the check
    cmdParseAddParam(Params[0], Func, Robot);
	return 0;
}

int cmdParseElseIf(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    BlockOffsetStruct *NewOffset;
    int i;
    
    //see if the current block is an IF block
    if(!CurrentBlock || CurrentBlock->Type != BLOCK_IF)
    {
        ErrorMsg = "Unexpected elseif";
        NEXT_STATE(PARSE_ERROR);
    }
    
    //add in a jump, first make a gap for it
    //the gap will be at the beginning of this line
    //i know a memmove would handle this *shrugs*
    for(i = Func->CmdLen; i >= OffsetAtLineStart; i--)
        Func->Cmds[i+3] = Func->Cmds[i];
    
    //add in the jump and update the length of our commands
    Func->Cmds[OffsetAtLineStart] = CMD_Jmp;
    Func->CmdLen += 3;
    
    //add in an entry for the jump so endif knows all jumps to update
    NewOffset = malloc(sizeof(BlockOffsetStruct));
    NewOffset->Offset = OffsetAtLineStart + 1;
    NewOffset->Next = CurrentBlock->ExtraOffsets;
    CurrentBlock->ExtraOffsets = NewOffset;
    
    //update the if statement to point to the beginning of this line if it fails
    *(short *)&Func->Cmds[CurrentBlock->Offset] = (OffsetAtLineStart + 3) - CurrentBlock->Offset;
    
    //update the block offset to update
    CurrentBlock->Offset = Func->CmdLen + 1;
    
    //setup a new if check
    Func->Cmds[Func->CmdLen] = CMD_If;
    Func->CmdLen += 3;
    
    cmdParseAddParam(Params[0], Func, Robot);
	return 0;
}

int cmdParseElse(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    BlockOffsetStruct *NewOffset;
    
    //see if the current block is an IF block
    if(!CurrentBlock || CurrentBlock->Type != BLOCK_IF)
    {
        ErrorMsg = "Unexpected else";
        NEXT_STATE(PARSE_ERROR);
    }
    
    //add in a jump
    Func->Cmds[Func->CmdLen] = CMD_Jmp;
    Func->CmdLen += 3;
    
    //add in an entry for the jump so endif knows all jumps to update
    NewOffset = malloc(sizeof(BlockOffsetStruct));
    NewOffset->Offset = Func->CmdLen - 2;
    NewOffset->Next = CurrentBlock->ExtraOffsets;
    CurrentBlock->ExtraOffsets = NewOffset;
    
    //update the if statement to point here if it fails
    *(short *)&Func->Cmds[CurrentBlock->Offset] = Func->CmdLen - CurrentBlock->Offset;

    //update the block
    CurrentBlock->Type = BLOCK_ELSE;
	return 0;
}

int cmdParseEndIf(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    BlockInfoStruct *CurBlock;
    BlockOffsetStruct *CurOffsetBlock;
    
    //see if the current block is an IF block
    if(!CurrentBlock || ((CurrentBlock->Type != BLOCK_IF) && (CurrentBlock->Type != BLOCK_ELSE)))
    {
        ErrorMsg = "Unexpected endif";
        NEXT_STATE(PARSE_ERROR);
    }
    
    //update the if statement to point here if it fails, else doesn't have an update to do
    if(CurrentBlock->Type == BLOCK_IF)
        *(short *)&Func->Cmds[CurrentBlock->Offset] = Func->CmdLen - CurrentBlock->Offset;

    //update all jumps
    while(CurrentBlock->ExtraOffsets)
    {
        CurOffsetBlock = CurrentBlock->ExtraOffsets;
        CurrentBlock->ExtraOffsets = CurOffsetBlock->Next;
        
        //update the jump entry to hit here
        *(short *)&Func->Cmds[CurOffsetBlock->Offset] = Func->CmdLen - CurOffsetBlock->Offset;
        free(CurOffsetBlock);
    }
    
    //remove the block entry
    CurBlock = CurrentBlock;
    CurrentBlock = CurrentBlock->ParentBlock;
    free(CurBlock);

    return 0;
}

int cmdParseWhile(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    //create a new block entry and add it
    BlockInfoStruct *NewBlock = malloc(sizeof(BlockInfoStruct));
    BlockOffsetStruct *NewOffset = malloc(sizeof(BlockOffsetStruct));
    
    NewBlock->Type = BLOCK_WHILE;
    NewBlock->ParentBlock = CurrentBlock;
    NewBlock->ExtraOffsets = NewOffset;
    NewBlock->Offset = OffsetAtLineStart;
    CurrentBlock = NewBlock;

    //very first entry of the offsets is where we jump to when the if statement fails
    NewOffset->Offset = Func->CmdLen + 1;
    NewOffset->Next = 0;
    
    //add in the if command that sees if we need to do anything
    Func->Cmds[Func->CmdLen] = CMD_If;
    Func->CmdLen += 3;
    
    //add our param
    cmdParseAddParam(Params[0], Func, Robot);
	return 0;
}

int cmdParseBreak(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    //add in a jump entry
    BlockOffsetStruct *NewOffset;
    BlockInfoStruct *CurBlock;

    //add in a jump entry to the end however add it for the while block
    CurBlock = CurrentBlock;
    while(CurBlock && CurBlock->Type != BLOCK_WHILE)
        CurBlock = CurBlock->ParentBlock;

    //if we hit the top of the list then error
    if(!CurBlock){
        ErrorMsg = "Unexpected break";
        NEXT_STATE(PARSE_ERROR);
    }

    //add in a jump
    Func->Cmds[Func->CmdLen] = CMD_Jmp;
    Func->CmdLen += 3;
    
    //add in an entry for the jump so endw knows all jumps to update
    NewOffset = malloc(sizeof(BlockOffsetStruct));
    NewOffset->Offset = Func->CmdLen - 2;
    NewOffset->Next = CurrentBlock->ExtraOffsets;
    CurrentBlock->ExtraOffsets = NewOffset;
	return 0;
}

int cmdParseNext(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    //add in a jump entry back to the beginning however add it for the while block
    BlockInfoStruct *CurBlock;
    
    CurBlock = CurrentBlock;
    while(CurBlock && CurBlock->Type != BLOCK_WHILE)
        CurBlock = CurBlock->ParentBlock;
    
    //if we hit the top of the list then error
    if(!CurBlock){
        ErrorMsg = "Unexpected next";
        NEXT_STATE(PARSE_ERROR);
    }
    
    //add in a jump for this entry
    Func->Cmds[Func->CmdLen] = CMD_Jmp;
    *(short *)&Func->Cmds[Func->CmdLen+1] = (short)(CurBlock->Offset - (Func->CmdLen + 1));
    Func->CmdLen += 3;
    
    return 0;
}

int cmdParseEndWhile(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    BlockInfoStruct *CurBlock;
    BlockOffsetStruct *CurOffsetBlock;
    
    //see if the current block is a while block
    if(!CurrentBlock || (CurrentBlock->Type != BLOCK_WHILE))
    {
        ErrorMsg = "Unexpected endw";
        NEXT_STATE(PARSE_ERROR);
    }
    
    //add in a jump to the beginning
    Func->Cmds[Func->CmdLen] = CMD_Jmp;
    *(short *)&Func->Cmds[Func->CmdLen+1] = CurrentBlock->Offset - (Func->CmdLen + 1);
    Func->CmdLen += 3;
    
    //update all jumps to go past the jump to the beginning
    while(CurrentBlock->ExtraOffsets)
    {
        CurOffsetBlock = CurrentBlock->ExtraOffsets;
        CurrentBlock->ExtraOffsets = CurOffsetBlock->Next;
        
        //update the jump entry to hit here
        *(short *)&Func->Cmds[CurOffsetBlock->Offset] = Func->CmdLen - CurOffsetBlock->Offset;
        free(CurOffsetBlock);
    }
    
    //remove the block entry
    CurBlock = CurrentBlock;
    CurrentBlock = CurrentBlock->ParentBlock;
    free(CurBlock);
    
    return 0;
}

int cmdParseAbs(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_Abs;
    Func->CmdLen++;
    
    cmdParseAddParam(Params[0], Func, Robot);
	return 0;
}

int cmdParseMax(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_Max;
    Func->CmdLen++;
    
    cmdParseAddParam(Params[0], Func, Robot);
	return 0;
}

int cmdParseMin(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_Min;
    Func->CmdLen++;
    
    cmdParseAddParam(Params[0], Func, Robot);
	return 0;
}

int cmdParseRound(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_Round;
    Func->CmdLen++;
    
    cmdParseAddParam(Params[0], Func, Robot);
	return 0;
}

int cmdParseTruncate(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_Truncate;
    Func->CmdLen++;
    
    cmdParseAddParam(Params[0], Func, Robot);
    return 0;
}

int cmdParseGetTime(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_GetTime;
    Func->CmdLen++;
    return 0;
}

int cmdParseGetRandom(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_GetRandom;
    Func->CmdLen++;
    return 0;
}

int cmdParseIsEqual(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    Func->Cmds[Func->CmdLen] = CMD_IsEqual;
    Func->CmdLen++;
    
    cmdParseAddParam(Params[0], Func, Robot);
    cmdParseAddParam(Params[1], Func, Robot);
    return 0;
}

int cmdParsePrint(char **Params[], FunctionStruct *Func, RobotStruct *Robot)
{
    int i;
    int len;
    
    //BUG: we fail to validate length is < 256 allowing arbitrary data into the buffer
    
    //get a param count
    for(i = 0; Params[i]; i++){}
    
    Func->Cmds[Func->CmdLen] = CMD_Print;
    Func->Cmds[Func->CmdLen+1] = i;
    Func->CmdLen += 2;

    for(i = 0; Params[i]; i++)
    {
        if(Params[i][0][0] == '"')
        {
            //output a string
            len = my_strlen(Params[i][0]);
            
            //remove the end "
            Func->Cmds[Func->CmdLen] = PARAM_STRING;
            Func->Cmds[Func->CmdLen+1] = (unsigned char)len - 2;
            memcpy(&Func->Cmds[Func->CmdLen+2], &(Params[i][0])[1], len - 2);
            Func->CmdLen += len;
        }
        else
            cmdParseAddParam(Params[i], Func, Robot);
    }
	return 0;
}
