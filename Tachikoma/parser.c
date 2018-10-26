#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "robotbattle.h"
#include "parsecmds.h"

char *GetStateName(enum StateEnum State)
{
    char *StateNames[] = {
        "PARSE_SWAP_STATE",
        "CHECK_INDENT",
        "PARSE_ERROR",
        "CMD_ERROR",
        "CHECK_COMMENT",
        "NEXT_LINE",
        "PARSE_COMMAND",
        "PARSE_FUNCTION",
        "PARSE_FUNCTION_START",
        "PARSE_PARAMS",
        "PARSE_PARAMS_MATH",
        "KILLED",
        "WAIT_ON_MOTION",
        "WAIT_ON_ROTATION",
        "IDLE_STATE",
        "NEXT_COMMAND",
        "NEXT_EVENT",
        "STALLED",
        "PARSE_DONE",
        "PARSE_ENDBLOCK",
        "PARSE_FUNCTION_END",
        "COMPILE_COMMAND",
        "PARSE_ASSIGNMENT",
        "PARSE_MATH",
        "PARSE_MATH_ANGLES",
        "PARSE_MATH_ADDSUB",
        "PARSE_MATH_MULDIV",
        "PARSE_MATH_EXPONENT"
    };
    
    if(((int)State < 0) || ((int)State > (sizeof(StateNames) / sizeof(char *))))
        return "UNKNOWN STATE";
    else
        return StateNames[State];
}

StateStruct ParseState;

typedef struct MathFuncStruct
{
    char *TextCmd;
    int ParamCount;
} MathFuncStruct;

RobotStruct *InitRobot()
{
    RobotStruct *NewRobot;
    NewRobot = (RobotStruct *)malloc(sizeof(RobotStruct));
    memset((void *)NewRobot, 0, sizeof(RobotStruct));
    NewRobot->SystemVars[_true] = ImmToMyDouble(1);
    NewRobot->SystemVars[_accelltarget] = ImmToMyDouble(MAX_ACCEL_SPEED);
    NewRobot->SystemVars[_deceltarget] = ImmToMyDouble(MAX_DECEL_SPEED);
    NewRobot->SystemVars[_speedtarget] = ImmToMyDouble(MAX_SPEED);
    NewRobot->SystemVars[_energy] = ImmToMyDouble(100);
    NewRobot->OldEnergy = NewRobot->SystemVars[_energy];
    return NewRobot;
}

volatile RobotStruct *ParseRobotData;
RobotStruct *ParseRobot(char *RobotCmds)
{
    volatile char **Lines;
    volatile int LineCount;
    volatile int LinePos;
    volatile int CurLineCount;
    volatile int CurLinePos;
    volatile int NewIndent;
    volatile int InsideBlock;
    volatile int ParseResult;
    volatile int MaxFunctionCount;
    volatile FunctionStruct **NewFunctions;
    volatile int TempVal, TempVal2;
    volatile int FunctionStart = 0;
    volatile int CurrentCmdSize;
    volatile int CurMathEntry;
    volatile int TempResultID;
    volatile int ParamCount;
    volatile int CurParamIndex;
    volatile int PrevCommaPos;
    volatile int ParenCount;
    volatile int BracketCount;
    volatile char **Params[10];
    volatile int i, x;
    typedef struct ForcedOrderStruct
    {
        char *CurLineData[100];    //sure hope no one ever tries to give us too manyspaces to parse
        char ErrorMsgBuf[256];
        StateStruct OrigState;
    } ForcedOrderStruct;
    volatile ForcedOrderStruct ForcedOrder;
    volatile int Done = 0;
    volatile int MultiLineComment = 0;
    volatile char **TempParamNames;
    volatile int InQuote;
    volatile int LastQuote;
    
    //backup the original state
    CREATE_GLOBAL_STATE(&ForcedOrder.OrigState);
    if(Done)
    {
            FREE_STATE(&ParseState);
            return ParseRobotData;
    }
    
    Done = 1;
    LineCount = SplitData(RobotCmds, 0, '\n', 0, -1);
    Lines = (volatile char **)malloc(sizeof(unsigned char **) * LineCount);
    ParseRobotData = InitRobot();

    //init the robot, turn on blocking and create enough space for 1 function per event
    //we'll expand the function list if need be
    
    ParseRobotData->Blocking = 1;
    ParseRobotData->Functions = 0;
    MaxFunctionCount = 0;
    
    //split up all of the lines
    LineCount = SplitData(RobotCmds, Lines, '\n', 0, LineCount);

    my_printf("Found %d lines\n", LineCount);

    //now work through each line
    InsideBlock = 0;
    NewIndent = 0;
    CurLinePos = 0;
    CurLineCount = 0;
    LinePos = 0;
    ParseResult = 0;
    CurrentBlock = 0;
    InQuote = -1;
    LastQuote = 0;
    TempParamNames = 0;
    
    memset(&ParseState, 0, sizeof(StateStruct));
    ParseState.CurState = CHECK_INDENT;
    
    SAVE_EVENT_STATE(&ParseState, 1);
        
    //my_printf("State (%d): %s\n", ParseState.CurState, GetStateName(ParseState.CurState));
    
    switch(ParseState.CurState)
    {
        case PARSE_ERROR:
            my_snprintf(ForcedOrder.ErrorMsgBuf, sizeof(ForcedOrder.ErrorMsgBuf), "Error during parse, Line %d - %s\n", LinePos+1, ErrorMsg);
            write(1, ForcedOrder.ErrorMsgBuf, strlen((char *)ForcedOrder.ErrorMsgBuf));
            ParseResult = -1;
            ParseRobotData->Initialized = -1;
            NEXT_STATE(PARSE_DONE);
            
        case CHECK_INDENT:
            if(Lines[LinePos][NewIndent] == 0x20)
                NewIndent++;
            else if(Lines[LinePos][NewIndent] == 0x09)
                NewIndent++;
            else
            {
                NEXT_STATE(CHECK_COMMENT);
            }    
            NEXT_STATE(CHECK_INDENT);

        case CHECK_COMMENT:
            if(Lines[LinePos][NewIndent] == '#')
                NEXT_STATE(NEXT_LINE);

            //multiline comment?
            if((Lines[LinePos][NewIndent] == '/') && (Lines[LinePos][NewIndent+1] == '*'))
            {
                MultiLineComment = 1;
                NEXT_STATE(NEXT_LINE);
            }
            
            if(MultiLineComment && (Lines[LinePos][NewIndent] == '*') && (Lines[LinePos][NewIndent+1] == '/'))
            {
                MultiLineComment = 0;
                NEXT_STATE(NEXT_LINE);
            }

            if(MultiLineComment)
                NEXT_STATE(NEXT_LINE);
            
            //not a comment, blank line?
            if(Lines[LinePos][NewIndent] == 0)
                NEXT_STATE(NEXT_LINE);
            
            //not a blank line, try to parse the command up
            NEXT_STATE(PARSE_COMMAND);
            
        case NEXT_LINE:
            NewIndent = 0;
            LinePos++;
            if(LinePos >= LineCount)
            {
                if(InsideBlock || FunctionStart)
                {
                    ErrorMsg = "End of file before end of block";
                    NEXT_STATE(PARSE_ERROR);
                }

                //done with the parsing                
                NEXT_STATE(PARSE_DONE);
            }
                
            NEXT_STATE(CHECK_INDENT);
            
        case PARSE_COMMAND:
            //potential overflow if too many spaces
                    memset(ForcedOrder.CurLineData, 0, sizeof(ForcedOrder.CurLineData));

            CurLineCount = SplitData((char *)&Lines[LinePos][NewIndent], ForcedOrder.CurLineData, ' ', 1, -1);
            if(NewIndent == 0)
            {
                if(InsideBlock)
                    NEXT_STATE(PARSE_FUNCTION_END);
                else if(FunctionStart)
                    NEXT_STATE(PARSE_FUNCTION_START);
                else
                    NEXT_STATE(PARSE_FUNCTION);
            }

            //we should be in the middle of a function if there are indents
            if(!InsideBlock)
            {
                ErrorMsg = "Commands found outside of a function";
                NEXT_STATE(PARSE_ERROR);
            }

            //store off where this line starts as some entries require knowing it
            OffsetAtLineStart = ParseRobotData->Functions[ParseRobotData->FunctionCount - 1]->CmdLen;

            //if we are here, see if the count is short, if so then we should have an end block
            if(CurLineCount == 1)
                NEXT_STATE(PARSE_ENDBLOCK);

            //see if we can find an = in the data, if so then we have an assignment otherwise assume a command
            for(TempVal = 0; (TempVal < CurLineCount) && (ForcedOrder.CurLineData[TempVal][0] != '='); TempVal++){}
            if(TempVal != CurLineCount)
            {
                //found an equal, make sure nothing is after it
                if(ForcedOrder.CurLineData[TempVal][1] == 0)
                    NEXT_STATE(PARSE_ASSIGNMENT);
            }
            
            //try to locate a line ending with a ( to indicate a command
            TempVal = strlen(ForcedOrder.CurLineData[0])-1;
            if(ForcedOrder.CurLineData[0][TempVal] != '(')
            {
                ErrorMsg = "Failed to find a command";
                NEXT_STATE(PARSE_ERROR);
            }

            //the ( is at the end, remove it
            ForcedOrder.CurLineData[0][TempVal] = 0;

            //verify that the last entry is the closing )
            TempVal = strlen(ForcedOrder.CurLineData[CurLineCount-1]);
            if(!TempVal || (ForcedOrder.CurLineData[CurLineCount-1][TempVal-1] != ')'))
            {
                ErrorMsg = "Failed to finding closing ) on command";
                NEXT_STATE(PARSE_ERROR);
            }
            
            //remove the )
            if(TempVal == 1)
                CurLineCount--;
            else
                ForcedOrder.CurLineData[CurLineCount-1][TempVal-1] = 0;

            //lowercase the string
            //LowerCase((char *)ForcedOrder.CurLineData[0]);
    
            //go through and generate a final param list
            CurLinePos = 0;
            CurMathEntry = 0;
            TempResultID = 0;
            ParamCount = 0;
            CurParamIndex = 0;
            ParenCount = 0;
            BracketCount = 0;
            PrevCommaPos = 1;
            LastQuote = 0;
            memset(Params, 0, sizeof(Params));
            
            //my_printf("CurLineCount: %d\n", CurLineCount);
            NEXT_STATE(PARSE_PARAMS);

        case PARSE_PARAMS:
            //parse up the params based on comma's, keeping tracking open and close parens
            //due to commands like tan2()
            
            //BUG: we do fail to check for more than 10 params...
            
            //advance to next entry in the line
            if(CurLinePos == (CurLineCount - 1))
            {
                //hit the end
                //if we have an open paren or open bracket then fail
                if(ParenCount)
                {
                    ErrorMsg = "Mismatched parentheses";
                    NEXT_STATE(PARSE_ERROR);
                }
                else if(BracketCount)
                {
                    ErrorMsg = "Mismatched bracket";
                    NEXT_STATE(PARSE_ERROR);
                }
                else if(InQuote != -1)
                {
                    ErrorMsg = "Mismatched quote";
                    NEXT_STATE(PARSE_ERROR);
                }
                else if(LastQuote)
                {
                    //setup new param entry and setup the next field as the beginning of the last command
                    //assuming we didn't hit the end
                    if(LastQuote != CurLineCount)
                        Params[CurParamIndex] = &ForcedOrder.CurLineData[LastQuote];
                }
                else if(CurLinePos != 0)
                {
                    //setup the last param
                    Params[CurParamIndex] = &ForcedOrder.CurLineData[PrevCommaPos];
                }
 
                //if we added anything then update the count
                ParamCount = CurParamIndex;
                if(Params[CurParamIndex])
                    ParamCount++;
                
                //make sure the end entry is null
                ForcedOrder.CurLineData[CurLinePos + 1] = 0;
                CurParamIndex = 0;
                CurLinePos = 0;
                
                //my_printf("Found %d params\n", ParamCount);
                NEXT_STATE(PARSE_PARAMS_MATH);
            }

            //not end of the params, check for entries
            CurLinePos++;
            
            //get length of the current entry
            i = my_strlen(ForcedOrder.CurLineData[CurLinePos]);
            if(InQuote != -1)
            {
                //inside a quote area, add the space back
                ForcedOrder.CurLineData[CurLinePos][-1] = ' ';
                
                //see if we have a " at the end
                if(ForcedOrder.CurLineData[CurLinePos][i-1] == '"')
                {
                    Params[CurParamIndex] = &ForcedOrder.CurLineData[InQuote];
                    CurParamIndex++;
                    
                    //shift all fields over in ForcedOrder.CurLineData so we can have an empty entry
                    for(i = CurLineCount; i > (CurLinePos+1); i--)
                        ForcedOrder.CurLineData[i] = ForcedOrder.CurLineData[i-1];
                    
                    //insert null and extend the line length
                    CurLinePos++;
                    CurLineCount++;
                    ForcedOrder.CurLineData[CurLinePos] = 0;

                    InQuote = -1;
                    LastQuote = CurLinePos + 1;
                }
                
                NEXT_STATE(PARSE_PARAMS);   //move on to the next state
            }
            
            //if no characters
            if(i == 0)
            {
                //we shouldn't be here
                ErrorMsg = "Blank entry during parse";
                NEXT_STATE(PARSE_ERROR);
            }
            if(i == 1)
            {
                if(ForcedOrder.CurLineData[CurLinePos][0] == '(')
                    ParenCount++;
                else if(ForcedOrder.CurLineData[CurLinePos][0] == ')')
                {
                    ParenCount--;
                    if(ParenCount < 0)
                    {
                        ErrorMsg = "Mismatched parentheses";
                        NEXT_STATE(PARSE_ERROR);
                    }
                }
                else if(ForcedOrder.CurLineData[CurLinePos][0] == ']')
                {
                    BracketCount--;
                    if(BracketCount < 0)
                    {
                        ErrorMsg = "Mismatched bracket";
                        NEXT_STATE(PARSE_ERROR);
                    }
                }
                else if((ForcedOrder.CurLineData[CurLinePos][0] == ',') && (ParenCount == 0) && (BracketCount == 0) && (InQuote == -1))
                {
                    //found a comma by itself and not within a () or [] block
                    LastQuote = 0;
                    
                    //param entries need to end with a null entry so null this entry out
                    ForcedOrder.CurLineData[CurLinePos] = 0;
                    Params[CurParamIndex] = &ForcedOrder.CurLineData[PrevCommaPos];
                    
                    //setup new param entry and setup the next field as the beginning of the comma
                    PrevCommaPos = CurLinePos + 1;
                    
                    //increment and continue as needed
                    CurParamIndex++;
                }
            }
            else
            {
                if((ForcedOrder.CurLineData[CurLinePos][i-1] == ',') && (ParenCount == 0) && (BracketCount == 0) && (InQuote == -1))
                {
                    //found a comma and not within a () or [] block

                    //remove the comma
                    ForcedOrder.CurLineData[CurLinePos][i-1] = 0;
                    
                    //shift all fields over in ForcedOrder.CurLineData so we can have an empty entry
                    for(i = CurLineCount; i > (CurLinePos+1); i--)
                        ForcedOrder.CurLineData[i] = ForcedOrder.CurLineData[i-1];

                    //insert null and extend the line length
                    CurLinePos++;
                    CurLineCount++;
                    ForcedOrder.CurLineData[CurLinePos] = 0;

                    //setup new param entry and setup the next field as the beginning of the comma
                    Params[CurParamIndex] = &ForcedOrder.CurLineData[PrevCommaPos];
                    PrevCommaPos = CurLinePos + 1;
                    CurParamIndex++;
                }
                else if((ForcedOrder.CurLineData[CurLinePos][i-1] == '(') && (i >= 4))    //cos(, sin(, etc
                    ParenCount++;
                else if(ForcedOrder.CurLineData[CurLinePos][0] == ']')
                {
                    BracketCount--;
                    if(BracketCount < 0)
                    {
                        ErrorMsg = "Mismatched bracket";
                        NEXT_STATE(PARSE_ERROR);
                    }
                    
                    //double check that this isn't a[ x ][ y ] setup
                    if(ForcedOrder.CurLineData[CurLinePos][1] == '[')
                    {
                        BracketCount++;
                        if(ForcedOrder.CurLineData[CurLinePos][2] != 0)
                        {
                            ErrorMsg = "Invalid data after opening bracket";
                            NEXT_STATE(PARSE_ERROR);
                        }
                    }
                    else if((ForcedOrder.CurLineData[CurLinePos][1] != 0) && (ForcedOrder.CurLineData[CurLinePos][1] != '.'))
                    {
                        ErrorMsg = "Invalid data after bracket";
                        NEXT_STATE(PARSE_ERROR);
                    }
                }
                else if(ForcedOrder.CurLineData[CurLinePos][i-1] == '[')
                    BracketCount++;
                else if(ForcedOrder.CurLineData[CurLinePos][0] == '"')
                {
                    if(InQuote != -1)
                    {
                        ErrorMsg = "Mismatching quote";
                        NEXT_STATE(PARSE_ERROR);
                    }
                    
                    //if this is not the first entry then assign the previous area as a param
                    if(LastQuote)
                    {
                        //shift all fields over in ForcedOrder.CurLineData so we can have an empty entry
                        for(x = CurLineCount; x > CurLinePos; x--)
                            ForcedOrder.CurLineData[x] = ForcedOrder.CurLineData[x-1];
                        
                        //insert null and extend the line length
                        CurLinePos++;
                        CurLineCount++;
                        ForcedOrder.CurLineData[CurLinePos-1] = 0;
                        
                        //setup new param entry and setup the next field as the beginning of the last command
                        Params[CurParamIndex] = &ForcedOrder.CurLineData[LastQuote];
                        CurParamIndex++;
                        LastQuote = 0;
                    }
                    
                    //see if we have a quote at the end
                    if(ForcedOrder.CurLineData[CurLinePos][i-1] == '"')
                    {
                        //ends with a quote
                        Params[CurParamIndex] = &ForcedOrder.CurLineData[CurLinePos];
                        CurParamIndex++;
                        
                        //shift all fields over in ForcedOrder.CurLineData so we can have an empty entry
                        for(i = CurLineCount; i > (CurLinePos+1); i--)
                            ForcedOrder.CurLineData[i] = ForcedOrder.CurLineData[i-1];
                        
                        //insert null and extend the line length
                        CurLinePos++;
                        CurLineCount++;
                        ForcedOrder.CurLineData[CurLinePos] = 0;
                        
                        InQuote = -1;
                        LastQuote = CurLinePos + 1;
                    }
                    else
                        InQuote = CurLinePos;   //assign where our quote starts
                }
            }
            NEXT_STATE(PARSE_PARAMS);
            
        case PARSE_PARAMS_MATH:
        {
            //go through each param entry found and if it is more than 1 entry then
            //create a temporary list to be parsed by the math area
            if(CurParamIndex == 0)
            {
                //first time through, create a bunch of temporary entries
                TempParamNames = malloc(sizeof(char *)*ParamCount);
                for(i = 0; i < ParamCount; i++)
                    TempParamNames[i] = CreateTempVar(0);
            }
            
            //if we hit the end then compile the command, we'll free everything after the command is compiled
            if(CurParamIndex >= ParamCount)
                NEXT_STATE(COMPILE_COMMAND);
            
            //if we have more than 1 parameter then we need to create a temp assignment
            if(Params[CurParamIndex][1] && (Params[CurParamIndex][0][0] != '"'))
            {
                //copy all entries to a temporary list
                    
                //get a count of entries
                for(i = 0; Params[CurParamIndex][i]; i++){}

                //allocate memory for the pointers
                char **TempParams = malloc(sizeof(char *)*(i+3));
                
                //setup our temp list
                TempParams[0] = TempParamNames[CurParamIndex];
                TempParams[1] = "=";
                
                //copy the rest of the params to the list
                memcpy(&TempParams[2], &Params[CurParamIndex][0], sizeof(char *)*i);
                TempParams[i+2] = 0;
                
                //do math assignment
                cmdParseAssignMath(TempParams, ParseRobotData->Functions[ParseRobotData->FunctionCount-1], ParseRobotData);
                EraseTempVars(ParamCount);
                free(TempParams);

                //update the param list to have the temporary result and nothing more
                Params[CurParamIndex][0] = TempParamNames[CurParamIndex];
                Params[CurParamIndex][1] = 0;
            }
            
            CurParamIndex++;
            NEXT_STATE(PARSE_PARAMS_MATH);
        }
            
        case COMPILE_COMMAND:
            //go find the command
            for(TempVal = 0; ParseCmds[TempVal].Name != 0; TempVal++)
            {
                if(my_strcmp((char *)ForcedOrder.CurLineData[0], ParseCmds[TempVal].Name) == 0)
                    break;
            }

            if(ParseCmds[TempVal].Name == 0)
            {
                EraseTempVars(0);
                free(TempParamNames);
                TempParamNames = 0;
                
                ErrorMsg = "Unable to find command";
                NEXT_STATE(PARSE_ERROR);
            }

            //now go check number of params
            if((ParseCmds[TempVal].ParamCount != 0xffffffff) && (ParseCmds[TempVal].ParamCount != ParamCount))
            {
                EraseTempVars(0);
                free(TempParamNames);
                TempParamNames = 0;
                
                ErrorMsg = "Invalid number of params";
                NEXT_STATE(PARSE_ERROR);
            }
            
            //found the right command, let it generate it's bit data
            ParseCmds[TempVal].Handler(Params, ParseRobotData->Functions[ParseRobotData->FunctionCount-1], ParseRobotData);
            
            //free our pointers
            EraseTempVars(0);
            if(TempParamNames)
            {
                free(TempParamNames);
                TempParamNames = 0;
            }
            NEXT_STATE(NEXT_LINE);
            
        case PARSE_ASSIGNMENT:
            //parse up the assignment, should be x = a + b or similar
            //make sure the = is on it's own entry then pass to the parsing function
            
            //first, find the =, the rest of it will be parsed by the PARSE_MATH area
            
            //we could be here due to finding an = or because we didn't find a ( to indicate a function
            if(ForcedOrder.CurLineData[TempVal][0] != '=')
            {
                //ok, we got here but didn't have the =, if the next entry doesn't start with it then
                //we need to error
                if((CurLineCount < 2) || (ForcedOrder.CurLineData[1][0] != '='))
                {
                    ErrorMsg = "Unable to determine if line is a function or assignment";
                    NEXT_STATE(PARSE_ERROR);
                }
                
                //found that the next entry is =, see if we need to adjust
                if(ForcedOrder.CurLineData[1][1] != 0)
                {
                    //we need to shift everything over and remove the = then insert our own = entry
                    for(TempVal = CurLineCount; TempVal > 1; TempVal--)
                        ForcedOrder.CurLineData[TempVal] = ForcedOrder.CurLineData[TempVal-1];
                    
                    //make the line longer
                    CurLineCount++;
                    
                    //add our = and remove it from the 3rd param
                    ForcedOrder.CurLineData[2]++;    //remove the =
                    ForcedOrder.CurLineData[1] = "=";
                }
            }
            else if(ForcedOrder.CurLineData[TempVal][1] != 0)
            {
                ErrorMsg = "Error in parsing assignment";
                NEXT_STATE(PARSE_ERROR);
            }
            
            //go parse math
            cmdParseAssignMath(ForcedOrder.CurLineData, ParseRobotData->Functions[ParseRobotData->FunctionCount-1], ParseRobotData);
            EraseTempVars(0);
            NEXT_STATE(NEXT_LINE);

        case PARSE_MATH:
            //we *should* have a <oper> b entries. However we may have
            //lines similar to a+b+(c*d) or a+b*c+d
            //so we need keep oper order proper
            //we do require spaces around everything
            //go find opers to combine
            
            //order of operations
            //cos{x}, sin{x}, tan{x}, acos{x}, asin{x}, atan{x}, atan2{x,y}
            //^
            //*, /, %
            //+, -
            
            if(CurLinePos == 1)
            {
                if(CurLineCount == 2)
                {
                    //we only have 2 entries left, we need to assign the result now

                    //assign one variable to another to finalize the math line we found
                    cmdParseAssignMath(ForcedOrder.CurLineData, ParseRobotData->Functions[ParseRobotData->FunctionCount-1], ParseRobotData);
                    NEXT_STATE(NEXT_LINE);
                }
                
                //on the first entry, we have more to parse, search for a ( and ) combo
                TempVal2 = 0;
                for(TempVal = 2; TempVal < CurLineCount; TempVal++)
                {
                    if(ForcedOrder.CurLineData[TempVal][0] == '(')
                    {
                        if(ForcedOrder.CurLineData[TempVal][1] == 0)
                        {
                            CurLinePos = TempVal;
                            NEXT_STATE(PARSE_MATH);
                        }
                        
                        ErrorMsg = "Error parsing formula";
                        NEXT_STATE(PARSE_ERROR);
                    }
                }
                
                //look for any of the special operators
                for(TempVal = 1; TempVal < CurLineCount; TempVal++)
                {
                    if( (strcmp((char *)ForcedOrder.CurLineData[TempVal], "cos") == 0) ||
                        (strcmp((char *)ForcedOrder.CurLineData[TempVal], "sin") == 0) ||
                        (strcmp((char *)ForcedOrder.CurLineData[TempVal], "tan") == 0) ||
                        (strcmp((char *)ForcedOrder.CurLineData[TempVal], "acos") == 0) ||
                        (strcmp((char *)ForcedOrder.CurLineData[TempVal], "asin") == 0) ||
                        (strcmp((char *)ForcedOrder.CurLineData[TempVal], "atan") == 0) ||
                        (strcmp((char *)ForcedOrder.CurLineData[TempVal], "atan2") == 0))
                    {
                        CurLinePos = TempVal;
                        NEXT_STATE(PARSE_MATH_ANGLES);
                    }                    
                }

                //no special math, look for exponent
                for(TempVal = 2; TempVal < CurLineCount; TempVal++)
                {
                    if(ForcedOrder.CurLineData[TempVal][0] == '^' && ForcedOrder.CurLineData[TempVal][1] == 0)
                    {
                        CurLinePos = TempVal-1;
                        NEXT_STATE(PARSE_MATH_EXPONENT);
                    }                    
                }

                //no special math, look for mul, divide, and mod
                for(TempVal = 2; TempVal < CurLineCount; TempVal++)
                {
                    if(((ForcedOrder.CurLineData[TempVal][0] == '*') ||
                        (ForcedOrder.CurLineData[TempVal][0] == '/') ||
                        (ForcedOrder.CurLineData[TempVal][0] == '%')) && ForcedOrder.CurLineData[TempVal][1] == 0)
                    {
                        CurLinePos = TempVal-1;
                        NEXT_STATE(PARSE_MATH_MULDIV);
                    }                    
                }

                //no special math, look for +/-
                for(TempVal = 2; TempVal < CurLineCount; TempVal++)
                {
                    if(((ForcedOrder.CurLineData[TempVal][0] == '+') ||
                        (ForcedOrder.CurLineData[TempVal][0] == '-')) && ForcedOrder.CurLineData[TempVal][1] == 0)
                    {
                        CurLinePos = TempVal-1;
                        NEXT_STATE(PARSE_MATH_ADDSUB);
                    }                    
                }

                //can't find anything, fail
                ErrorMsg = "Error parsing expression";
                NEXT_STATE(PARSE_ERROR);
            }
            else if(ForcedOrder.CurLineData[CurLinePos][0] == '(')
            {
                //found a (, if 2nd entry is a ) or the 4th is a ) then we can process
                //otherwise we need to see if there is another ( before a )
                if(((CurLinePos+2) < CurLineCount) && (ForcedOrder.CurLineData[CurLinePos+2][0] == ')'))
                {
                    //we have ( a )
                    //remove the ( and ) from the array entries leaving just the a entry
                    ForcedOrder.CurLineData[CurLinePos] = ForcedOrder.CurLineData[CurLinePos+1];
                    for(TempVal = CurLinePos+1; TempVal < CurLineCount-2; TempVal++)
                        ForcedOrder.CurLineData[TempVal] = ForcedOrder.CurLineData[TempVal+2];
                    CurLineCount -= 2;

                    //start parsing from the beginning again
                    CurLinePos = 1;
                }
                else if(((CurLinePos+4) < CurLineCount) && (ForcedOrder.CurLineData[CurLinePos+4][0] == ')'))
                {
                    //we have ( a + b ) or similar, align on the a and reparse
                    CurLinePos++;
                    NEXT_STATE(PARSE_MATH);
                }
                else
                {
                    //go scan for another ( without a )
                    for(TempVal = CurLinePos + 1; TempVal < CurLineCount; TempVal++)
                    {
                        //the 2nd and 4th entry were not a ) so we should find a ( before another )
                        if(ForcedOrder.CurLineData[TempVal][0] == '(')
                        {
                            if(ForcedOrder.CurLineData[TempVal][1] == 0)
                            {
                                CurLinePos = TempVal;
                                NEXT_STATE(PARSE_MATH);
                            }
                        
                            ErrorMsg = "Error parsing formula";
                            NEXT_STATE(PARSE_ERROR);
                        }
                        else if(ForcedOrder.CurLineData[TempVal][0] == ')')
                            break;    
                    }
                    
                    //didn't find what we wanted
                    ErrorMsg = "Mismatched ( and )";
                    NEXT_STATE(PARSE_ERROR);
                }
            }

            //if we got here then we have issues as we couldn't combine the math
            ErrorMsg = "Error parsing formula";
            NEXT_STATE(PARSE_ERROR);

        case PARSE_FUNCTION:
            //should be the beginning of a function with a name
            if(CurLineCount != 1)
            {
                ErrorMsg = "Extra data found at beginning/end of function";
                NEXT_STATE(PARSE_ERROR);
            }

            //make this lower case to validate with
            LowerCase(ForcedOrder.CurLineData[0]);

            //line count is ok, see if the function is between a-z
            if(((ForcedOrder.CurLineData[0][0] | 0x40) < 'a') || ((ForcedOrder.CurLineData[0][0] | 0x40) > 'z'))
            {
                ErrorMsg = "Function name invalid";
                NEXT_STATE(PARSE_ERROR);
            }

            //we have the name of a function, create a new event for it
            if(ParseRobotData->FunctionCount >= (MaxFunctionCount-1))
            {
                MaxFunctionCount += 10;
                NewFunctions = (FunctionStruct **)malloc(MaxFunctionCount * sizeof(FunctionStruct *));

                if(ParseRobotData->Functions)
                {
                    memcpy((void *)NewFunctions, ParseRobotData->Functions, ParseRobotData->FunctionCount * sizeof(FunctionStruct *));
                    free(ParseRobotData->Functions);
                }
                memset((void *)&NewFunctions[ParseRobotData->FunctionCount], 0, (MaxFunctionCount - ParseRobotData->FunctionCount) * sizeof(FunctionStruct *));
                ParseRobotData->Functions = NewFunctions;
            }
            
            //setup the new function
            TempVal = strlen((char *)ForcedOrder.CurLineData[0]);
            ParseRobotData->Functions[ParseRobotData->FunctionCount] = (FunctionStruct *)malloc(sizeof(FunctionStruct));
            memset((void *)ParseRobotData->Functions[ParseRobotData->FunctionCount], 0, sizeof(FunctionStruct));

            //copy the name over
            ParseRobotData->Functions[ParseRobotData->FunctionCount]->Name = (char *)malloc(TempVal + 1);
            memcpy(ParseRobotData->Functions[ParseRobotData->FunctionCount]->Name, (void *)ForcedOrder.CurLineData[0], TempVal);
            ParseRobotData->Functions[ParseRobotData->FunctionCount]->Name[TempVal] = 0;

            //setup a default 4k buffer for the commands
            ParseRobotData->Functions[ParseRobotData->FunctionCount]->Cmds = (char *)malloc(4096);
            CurrentCmdSize = 4096;
            
            //setup the next function
            ParseRobotData->FunctionCount++;
            FunctionStart = 1;
            NEXT_STATE(NEXT_LINE);
            
        case PARSE_FUNCTION_START:
            FunctionStart = 0;
            
            //should be the beginning of a function with a {
            if(CurLineCount != 1)
            {
                ErrorMsg = "Extra data found at beginning of function";
                NEXT_STATE(PARSE_ERROR);
            }
            
            //line count is ok, see if we found something that isn't a {
            if((ForcedOrder.CurLineData[0][0] != '{') || (ForcedOrder.CurLineData[0][1] != 0))
            {
                ErrorMsg = "Extra data found at beginning of function";
                NEXT_STATE(PARSE_ERROR);
            }

            //beginning of function, see if we should have the beginning of the function
            if(InsideBlock != 0)
            {
                ErrorMsg = "Found beginning of function without ending previous function";
                NEXT_STATE(PARSE_ERROR);
            }

            InsideBlock++;

            //go do a new line
            NEXT_STATE(NEXT_LINE);

        case PARSE_FUNCTION_END:
            //should be the beginning of a function with a name
            if(CurLineCount != 1)
            {
                ErrorMsg = "Extra data found at end of function";
                NEXT_STATE(PARSE_ERROR);
            }
            
            //line count is ok, see if we found something that isn't a }
            if((ForcedOrder.CurLineData[0][0] != '}') || (ForcedOrder.CurLineData[0][1] != 0))
            {
                ErrorMsg = "Extra data found at end of function";
                NEXT_STATE(PARSE_ERROR);
            }

            //end of function, see if we should have the end of the function
            if(InsideBlock == 0)
            {
                ErrorMsg = "Found end of function without beginning";
                NEXT_STATE(PARSE_ERROR);
            }

            //check our list of internal blocks of the function
            if(CurrentBlock)
            {
                ErrorMsg = "Found end of function too soon";
                NEXT_STATE(PARSE_ERROR);
            }

            //go do a new line
            InsideBlock = 0;
            NEXT_STATE(NEXT_LINE);
            
        case PARSE_ENDBLOCK:
            //we should be inside of a block, verify we are inside of a block
            //and got the correct closing tag
            
            //this should be an endif, endwhile, etc
            if(my_strcmp(ForcedOrder.CurLineData[0],"endw") && my_strcmp(ForcedOrder.CurLineData[0],"endif") &&
               my_strcmp(ForcedOrder.CurLineData[0], "else") && my_strcmp(ForcedOrder.CurLineData[0], "return") &&
               my_strcmp(ForcedOrder.CurLineData[0], "next") && my_strcmp(ForcedOrder.CurLineData[0], "break"))
            {
                ErrorMsg = "Unknown end of block command";
                NEXT_STATE(PARSE_ERROR);
            }

            //make sure no data after it
            if(CurLineCount != 1)
            {
                ErrorMsg = "Line too long for end of block";
                NEXT_STATE(PARSE_ERROR);
            }
            
            //pass it off to be handled appropriately, the code will check and handle the block area to make sure it is valid
            //we have to call compile_command as the normal route expects () on the function
            ParamCount = 0;
            NEXT_STATE(COMPILE_COMMAND);
            
        case PARSE_DONE:
            SWAP_TO_STATE(&ForcedOrder.OrigState);
    };
    
    SWAP_TO_STATE(&ForcedOrder.OrigState);
}
