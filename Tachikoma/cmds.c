#include <math.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <fcntl.h>
#include <memory.h>
#include <unistd.h>
#include "robotbattle.h"
#include "cmds.h"
#include "time.h"

#define CMDFUNC(FUNC) void FUNC(RobotStruct *RobotData, FunctionStruct *Func)

#define GETVAR() GetVariable(RobotData, Func, 0)
#define SETVAR(NewVal) SetVariable(RobotData, Func, NewVal, 0)
#define GETFUNCID() GetFuncID(RobotData, Func)
MyDouble GetVariable(RobotStruct *RobotData, FunctionStruct *Func, GlobalVarStruct *GlobalEntry)
{
    MyDouble Val;
    int CurEntry;
    int NameLen;
    char Property[31];
    GlobalVarStruct *NewEntry;
    
    switch((unsigned char)Func->Cmds[Func->CmdPos])
    {
        case PARAM_NUM:
            //just the next value as a MyDouble
            Val = *(MyDouble *)&Func->Cmds[Func->CmdPos+1];
            Func->CmdPos += 1 + sizeof(struct MyDouble);
            break;
            
        case PARAM_SYSTEM:
            //system variable
            Val = RobotData->SystemVars[(unsigned char)Func->Cmds[Func->CmdPos+1]];
            Func->CmdPos += 2;
            break;
            
        case PARAM_GLOBAL:
            //global robot variable
            Val = RobotData->GlobalVars[(unsigned char)Func->Cmds[Func->CmdPos+1]].Value;
            Func->CmdPos += 2;
            break;
            
        case PARAM_TEMP:
            //temporary variable
            Val = RobotData->TempVars[(unsigned char)Func->Cmds[Func->CmdPos+1]];
            Func->CmdPos += 2;
            break;
            
        case PARAM_GLOBAL_SUB:
            //select the global entry then get our sub area
            GlobalEntry = &RobotData->GlobalVars[(unsigned char)Func->Cmds[Func->CmdPos+1]];
            Func->CmdPos += 2;
            Func->CmdPos %= Func->CmdLen;
            return GetVariable(RobotData, Func, GlobalEntry);
            
        case PARAM_SUBFINAL:
            //hit the end of this chain so stop processing
            Func->CmdPos++;
            Val = GlobalEntry->Value;
            break;
            
        case PARAM_SUBARRAY:
            //the sub entry is another value so look it up
            Func->CmdPos++;
            Func->CmdPos %= Func->CmdLen;

            //BUG: We don't validate negative indexing
            CurEntry = GetVariable(RobotData, Func, GlobalEntry).SubNumber;
            if(CurEntry >= GlobalEntry->ArraySize)
            {
                //resize the array
                NewEntry = malloc(sizeof(GlobalVarStruct)*(CurEntry+1));
                memcpy(NewEntry, GlobalEntry->Array, sizeof(GlobalVarStruct)*GlobalEntry->ArraySize);
                memset(&NewEntry[GlobalEntry->ArraySize], 0, sizeof(GlobalVarStruct)*((CurEntry+1)-CurEntry-GlobalEntry->ArraySize));

                //now put things in place and update the array size
                if(GlobalEntry->Array)
                    free(GlobalEntry->Array);
                GlobalEntry->Array = NewEntry;
                GlobalEntry->ArraySize = CurEntry+1;
            }
            GlobalEntry = &GlobalEntry->Array[CurEntry];
            return GetVariable(RobotData, Func, GlobalEntry);
            
        case PARAM_SUBPROP:
            //sub property of an indexed area that we can't guarantee so look it up
            //we have a few abuses that will let someone rewrite the stream so we are assuming that our normal property check is ok
            NameLen = Func->Cmds[Func->CmdPos+1];
            memcpy(Property, &Func->Cmds[Func->CmdPos+2], NameLen);
            Func->CmdPos += NameLen + 2;
            Func->CmdPos %= Func->CmdLen;
            Property[NameLen] = 0;
            
            for(CurEntry = 0; CurEntry < GlobalEntry->PropSize; CurEntry++)
            {
                if(my_strcmp(GlobalEntry->Prop[CurEntry].Name, Property) == 0)
                    break;
            }
            
            if(CurEntry != GlobalEntry->PropSize)
            {
                GlobalEntry = &GlobalEntry->Prop[CurEntry];
                return GetVariable(RobotData, Func, GlobalEntry);
            }
            
            //unable to find the sub property, add it to the entry
            NewEntry = malloc(sizeof(GlobalVarStruct)*(GlobalEntry->PropSize+1));
            memcpy(NewEntry, GlobalEntry->Prop, sizeof(GlobalVarStruct)*GlobalEntry->PropSize);
            CurEntry = GlobalEntry->PropSize;
            GlobalEntry->PropSize++;
            memset(&NewEntry[CurEntry], 0, sizeof(GlobalVarStruct));
            NewEntry[CurEntry].Name = malloc(NameLen+1);
            memcpy(NewEntry[CurEntry].Name, Property, NameLen);
            NewEntry[CurEntry].Name[NameLen] = 0;

            //free the old and put the new one in place
            if(GlobalEntry->Prop)
                free(GlobalEntry->Prop);
            GlobalEntry->Prop = NewEntry;
            GlobalEntry = &GlobalEntry->Prop[GlobalEntry->PropSize-1];
            return GetVariable(RobotData, Func, GlobalEntry);
            
        case PARAM_SUBARRAY_NUM:
            //BUG: we don't validate negative indexing
            GlobalEntry = &GlobalEntry->Array[*(int *)&Func->Cmds[Func->CmdPos+1]];
            Func->CmdPos += 1 + sizeof(int);
            Func->CmdPos %= Func->CmdLen;
            return GetVariable(RobotData, Func, GlobalEntry);
            
        case PARAM_SUBPROP_NUM:
            GlobalEntry = &GlobalEntry->Prop[*(unsigned char *)&Func->Cmds[Func->CmdPos+1]];
            Func->CmdPos += 2;
            Func->CmdPos %= Func->CmdLen;
            return GetVariable(RobotData, Func, GlobalEntry);
            
        default:
            ErrorMsg = "Unknown variable request";
            NEXT_STATE(CMD_ERROR);
    }
    
    Func->CmdPos %= Func->CmdLen;
    return Val;
}

int GetFuncID(RobotStruct *RobotData, FunctionStruct *Func)
{
    //get the function ID
    int FuncID;
    FuncID = Func->Cmds[Func->CmdPos];
    Func->CmdPos++;
    Func->CmdPos %= Func->CmdLen;
    return FuncID;
}
void SetVariable(RobotStruct *RobotData, FunctionStruct *Func, MyDouble NewVal, GlobalVarStruct *GlobalEntry)
{
    int CurEntry;
    int NameLen;
    char Property[31];
    GlobalVarStruct *NewEntry;

    switch((unsigned char)Func->Cmds[Func->CmdPos])
    {
        case PARAM_GLOBAL:
            //global robot variable
            RobotData->GlobalVars[(unsigned char)Func->Cmds[Func->CmdPos+1]].Value = NewVal;
            Func->CmdPos += 2;
            break;
            
        case PARAM_TEMP:
            //temporary variable
            RobotData->TempVars[(unsigned char)Func->Cmds[Func->CmdPos+1]] = NewVal;
            Func->CmdPos += 2;
            break;
            
            //still need to add the other param types that handle user types
        case PARAM_GLOBAL_SUB:
            //select the global entry then get our sub area
            GlobalEntry = &RobotData->GlobalVars[(unsigned char)Func->Cmds[Func->CmdPos+1]];
            Func->CmdPos += 2;
            SetVariable(RobotData, Func, NewVal, GlobalEntry);
            break;
            
        case PARAM_SUBFINAL:
            //hit the end of this chain so stop processing
            Func->CmdPos++;
            GlobalEntry->Value = NewVal;
            break;
            
        case PARAM_SUBARRAY:
            //the sub entry is another value so look it up
            Func->CmdPos++;
            CurEntry = GetVariable(RobotData, Func, GlobalEntry).SubNumber;
            if(CurEntry >= GlobalEntry->ArraySize)
            {
                //resize the array
                NewEntry = malloc(sizeof(GlobalVarStruct)*(CurEntry+1));
                memcpy(NewEntry, GlobalEntry->Array, sizeof(GlobalVarStruct)*GlobalEntry->ArraySize);
                memset(&NewEntry[GlobalEntry->ArraySize], 0, sizeof(GlobalVarStruct)*(CurEntry-GlobalEntry->ArraySize));
                
                //now put things in place and update the array size
                if(GlobalEntry->Array)
                    free(GlobalEntry->Array);
                GlobalEntry->Array = NewEntry;
                GlobalEntry->ArraySize = CurEntry+1;
            }
            GlobalEntry = &GlobalEntry->Array[CurEntry];
            SetVariable(RobotData, Func, NewVal, GlobalEntry);
            break;
            
        case PARAM_SUBPROP:
            //sub property of an indexed area that we can't guarantee so look it up
            NameLen = Func->Cmds[Func->CmdPos+1];
            memcpy(Property, &Func->Cmds[Func->CmdPos+2], NameLen);
            Func->CmdPos += NameLen + 2;
            Property[NameLen] = 0;
            
            for(CurEntry = 0; CurEntry < GlobalEntry->PropSize; CurEntry++)
            {
                if(my_strcmp(GlobalEntry->Prop[CurEntry].Name, Property) == 0)
                    break;
            }
            
            if(CurEntry != GlobalEntry->PropSize)
            {
                GlobalEntry = &GlobalEntry->Prop[CurEntry];
                SetVariable(RobotData, Func, NewVal, GlobalEntry);
            }
            
            //unable to find the sub property, add it to the entry
            NewEntry = malloc(sizeof(GlobalVarStruct)*(GlobalEntry->PropSize+1));
            memcpy(NewEntry, GlobalEntry->Prop, sizeof(GlobalVarStruct)*GlobalEntry->PropSize);
            CurEntry = GlobalEntry->PropSize;
            GlobalEntry->PropSize++;
            memset(&NewEntry[CurEntry], 0, sizeof(GlobalVarStruct));
            NewEntry[CurEntry].Name = malloc(NameLen+1);
            memcpy(NewEntry[CurEntry].Name, Property, NameLen);
            NewEntry[CurEntry].Name[NameLen] = 0;
            
            //free the old and put the new one in place
            if(GlobalEntry->Prop)
                free(GlobalEntry->Prop);
            GlobalEntry->Prop = NewEntry;
            GlobalEntry = &GlobalEntry->Prop[GlobalEntry->PropSize-1];
            SetVariable(RobotData, Func, NewVal, GlobalEntry);
            break;
            
        case PARAM_SUBARRAY_NUM:
            GlobalEntry = &GlobalEntry->Array[*(int *)&Func->Cmds[Func->CmdPos+1]];
            Func->CmdPos += 1 + sizeof(int);
            SetVariable(RobotData, Func, NewVal, GlobalEntry);
            break;
            
        case PARAM_SUBPROP_NUM:
            GlobalEntry = &GlobalEntry->Prop[*(unsigned char *)&Func->Cmds[Func->CmdPos+1]];
            Func->CmdPos += 2;
            SetVariable(RobotData, Func, NewVal, GlobalEntry);
            break;
            
        default:
            ErrorMsg = "Unknown variable to set";
            NEXT_STATE(CMD_ERROR);
    }
    
    Func->CmdPos %= Func->CmdLen;
}

CMDFUNC(cmdAssignMath)
{
    //a = b
    MyDouble Var1;
    
    Var1 = GETVAR();
    SETVAR(Var1);
}

CMDFUNC(cmdName)
{
}

CMDFUNC(cmdAhead)
{
    //attempt to move forward
    RobotData->SystemVars[_movermn] = MyDouble_ADD(RobotData->SystemVars[_movermn], GETVAR());
    RobotData->SystemVars[_moving].SubNumber = 1;
    
    if(RobotData->Blocking)
        NEXT_STATE(WAIT_ON_MOTION);
}

CMDFUNC(cmdBack)
{
    //attempt to move backwards
    RobotData->SystemVars[_movermn] = MyDouble_SUB(RobotData->SystemVars[_movermn], GETVAR());
    RobotData->SystemVars[_moving].SubNumber = 1;

    if(RobotData->Blocking)
        NEXT_STATE(WAIT_ON_MOTION);
}

CMDFUNC(cmdAccelTarget)
{
    //change the acceleration of the robot
    RobotData->SystemVars[_accelltarget] = GETVAR();
    if(MyDouble_LESSTHAN_EQUAL(RobotData->SystemVars[_accelltarget], MyDouble0))
    {
        RobotData->SystemVars[_accelltarget].SubNumber = 0;
        RobotData->SystemVars[_accelltarget].Fraction = 1;
        RobotData->SystemVars[_accelltarget].Signed = 0;
    }
    else if(MyDouble_GREATERTHAN(RobotData->SystemVars[_accelltarget], ImmToMyDouble(MAX_ACCEL_SPEED)))
    {
        RobotData->SystemVars[_accelltarget].SubNumber = MAX_ACCEL_SPEED;
        RobotData->SystemVars[_accelltarget].Fraction = 0;
        RobotData->SystemVars[_accelltarget].Signed = 0;
    }
}

CMDFUNC(cmdDecelTarget)
{
    //change the deceleration of the robot
    RobotData->SystemVars[_deceltarget] = GETVAR();
    if(MyDouble_LESSTHAN_EQUAL(RobotData->SystemVars[_deceltarget], MyDouble0))
    {
        RobotData->SystemVars[_deceltarget].SubNumber = 0;
        RobotData->SystemVars[_deceltarget].Fraction = 1;
        RobotData->SystemVars[_deceltarget].Signed = 0;
    }
    else if(MyDouble_GREATERTHAN(RobotData->SystemVars[_deceltarget], ImmToMyDouble(MAX_DECEL_SPEED)))
    {
        RobotData->SystemVars[_deceltarget].SubNumber = MAX_DECEL_SPEED;
        RobotData->SystemVars[_deceltarget].Fraction = 0;
        RobotData->SystemVars[_deceltarget].Signed = 0;
    }
}

CMDFUNC(cmdSpeedTarget)
{
    //change the target speed of the robot
    RobotData->SystemVars[_speedtarget] = GETVAR();
    if(MyDouble_LESSTHAN_EQUAL(RobotData->SystemVars[_speedtarget], MyDouble0))
    {
        RobotData->SystemVars[_speedtarget].SubNumber = 0;
        RobotData->SystemVars[_speedtarget].Fraction = 1;
        RobotData->SystemVars[_speedtarget].Signed = 0;
    }
    else if(MyDouble_GREATERTHAN(RobotData->SystemVars[_speedtarget], ImmToMyDouble(MAX_SPEED)))
    {
        RobotData->SystemVars[_speedtarget].SubNumber = MAX_SPEED;
        RobotData->SystemVars[_speedtarget].Fraction = 0;
        RobotData->SystemVars[_speedtarget].Signed = 0;
    }
}

CMDFUNC(cmdBodyLeft)
{
    //change the angle of the robot
    MyDouble Rotation;

    Rotation = MyDouble_MOD(GETVAR(), ImmToMyDouble(360));
    Rotation = MyDouble_SUB(MyDouble0, Rotation);
    RobotData->SystemVars[_bodyrmn] = Rotation;
    
    if(RobotData->RotationLock & ROTATE_LOCK_BODY)
    {
        RobotData->SystemVars[_gunrmn] = Rotation;
        RobotData->SystemVars[_radarrmn] = Rotation;
    }

    RobotData->SystemVars[_rotating].SubNumber = 1;

    if(RobotData->Blocking)
        NEXT_STATE(WAIT_ON_ROTATION);
}

CMDFUNC(cmdBodyRight)
{
    //change the angle of the robot
    MyDouble Rotation;
    
    Rotation = MyDouble_MOD(GETVAR(), ImmToMyDouble(360));
    RobotData->SystemVars[_bodyrmn] = Rotation;
    
    if(RobotData->RotationLock & ROTATE_LOCK_BODY)
    {
        RobotData->SystemVars[_gunrmn] = Rotation;
        RobotData->SystemVars[_radarrmn] = Rotation;
    }
    
    RobotData->SystemVars[_rotating].SubNumber = 1;

    if(RobotData->Blocking)
        NEXT_STATE(WAIT_ON_ROTATION);
}

CMDFUNC(cmdGunLeft)
{
    //change the angle of the robot
    MyDouble Rotation;
    
    Rotation = GETVAR();
    Rotation = MyDouble_MOD(Rotation, ImmToMyDouble(360));
    Rotation = MyDouble_SUB(MyDouble0, Rotation);
    RobotData->SystemVars[_gunrmn] = Rotation;
    
    if(RobotData->RotationLock & ROTATE_LOCK_BODY)
    {
        RobotData->SystemVars[_bodyrmn] = Rotation;
        RobotData->SystemVars[_radarrmn] = Rotation;
    }
    else if(RobotData->RotationLock & ROTATE_LOCK_RADAR)
        RobotData->SystemVars[_radarrmn] = Rotation;
    
    RobotData->SystemVars[_rotating].SubNumber = 1;

    if(RobotData->Blocking)
        NEXT_STATE(WAIT_ON_ROTATION);
}

CMDFUNC(cmdGunRight)
{
    //change the angle of the robot
    MyDouble Rotation;
    
    Rotation = MyDouble_MOD(GETVAR(), ImmToMyDouble(360));
    RobotData->SystemVars[_gunrmn] = Rotation;
    
    if(RobotData->RotationLock & ROTATE_LOCK_BODY)
    {
        RobotData->SystemVars[_bodyrmn] = Rotation;
        RobotData->SystemVars[_radarrmn] = Rotation;
    }
    else if(RobotData->RotationLock & ROTATE_LOCK_RADAR)
        RobotData->SystemVars[_radarrmn] = Rotation;

    RobotData->SystemVars[_rotating].SubNumber = 1;

    if(RobotData->Blocking)
        NEXT_STATE(WAIT_ON_ROTATION);
}

CMDFUNC(cmdRadarLeft)
{
    //change the angle of the robot
    MyDouble Rotation;

    Rotation = MyDouble_MOD(GETVAR(), ImmToMyDouble(360));
    Rotation = MyDouble_SUB(MyDouble0, Rotation);

    RobotData->SystemVars[_radarrmn] = Rotation;
    
    if(RobotData->RotationLock & ROTATE_LOCK_BODY)
    {
        RobotData->SystemVars[_bodyrmn] = Rotation;
        RobotData->SystemVars[_gunrmn] = Rotation;
    }
    else if(RobotData->RotationLock & ROTATE_LOCK_RADAR)
        RobotData->SystemVars[_gunrmn] = Rotation;

    
    RobotData->SystemVars[_rotating].SubNumber = 1;

    if(RobotData->Blocking)
        NEXT_STATE(WAIT_ON_ROTATION);
}

CMDFUNC(cmdRadarRight)
{
    //change the angle of the robot
    MyDouble Rotation;
    
    Rotation = MyDouble_MOD(GETVAR(), ImmToMyDouble(360));
    RobotData->SystemVars[_radarrmn] = Rotation;
    
    if(RobotData->RotationLock & ROTATE_LOCK_BODY)
    {
        RobotData->SystemVars[_bodyrmn] = Rotation;
        RobotData->SystemVars[_gunrmn] = Rotation;
    }
    else if(RobotData->RotationLock & ROTATE_LOCK_RADAR)
        RobotData->SystemVars[_gunrmn] = Rotation;
    
    RobotData->SystemVars[_rotating].SubNumber = 1;

    if(RobotData->Blocking)
        NEXT_STATE(WAIT_ON_ROTATION);
}

CMDFUNC(cmdStop)
{
    //stop immediate rotation, slow down the robot motion
    RobotData->StopMotion = 1;
    RobotData->StopRotation = 1;
}

CMDFUNC(cmdStopMove)
{
    //slow down the robot motion
    RobotData->StopMotion = 1;
}

CMDFUNC(cmdStopRotate)
{
    //stop immediate rotation
    RobotData->StopRotation = 1;
}

CMDFUNC(cmdScan)
{
    RobotData->ScanActive = 1;
}

CMDFUNC(cmdFire)
{
    //if the gun isn't hot then allow it to be fired if the strength is between 1 and 7
    MyDouble FireStrength = GETVAR();

    if(MyDouble_LESSTHAN_EQUAL(FireStrength, MyDouble0))
        return;
        
    if(MyDouble_GREATERTHAN_EQUAL(FireStrength, ImmToMyDouble(7)))
    {
        FireStrength = ImmToMyDouble(7);
    }

    if(MyDouble_EQUAL(RobotData->SystemVars[_gunheat], MyDouble0) && MyDouble_GREATERTHAN(RobotData->SystemVars[_energy], FireStrength))
    {
        MissileStruct *NewMissile;

        //create a missle and add it to our list
        NewMissile = malloc(sizeof(MissileStruct));
        NewMissile->Energy = MyDouble_MUL(FireStrength, ImmToMyDouble(4));
        NewMissile->Heading = RobotData->SystemVars[_gunaim];
        
        //remove the energy used
        RobotData->SystemVars[_energy] = MyDouble_SUB(RobotData->SystemVars[_energy], FireStrength);

        //setup the X/Y location, it should spawn just outside of the current robot
        NewMissile->X = MyDouble_ADD(RobotData->SystemVars[_xpos], DoubleToMyDouble(sin(MyDoubleToDouble(NewMissile->Heading) * DEG_TO_RADIAN) * ROBOT_SIZE));
        NewMissile->Y = MyDouble_ADD(RobotData->SystemVars[_ypos], DoubleToMyDouble(cos(MyDoubleToDouble(NewMissile->Heading) * DEG_TO_RADIAN) * ROBOT_SIZE));

    //my_printf("Missile: X/Y %f/%f heading %f energy %f, robot X/Y %f/%f\n", NewMissile->X, NewMissile->Y, NewMissile->Heading, NewMissile->Energy, RobotData->SystemVars[_xpos], RobotData->SystemVars[_ypos]);
//my_printf("Missile: %f + %f = %f\n", RobotData->SystemVars[_xpos], DoubleToMyDouble(sin(MyDoubleToDouble(NewMissile->Heading) * DEG_TO_RADIAN) * ROBOT_SIZE), MyDouble_ADD(RobotData->SystemVars[_xpos], DoubleToMyDouble(sin(MyDoubleToDouble(NewMissile->Heading) * DEG_TO_RADIAN) * ROBOT_SIZE)));
        
        //now add it to the list
        NewMissile->Next = Missiles;
        Missiles = NewMissile;
        
        //update gun heat
        RobotData->SystemVars[_gunheat] = MyDouble_ADD(MyDouble_MUL(ImmToMyDouble(2), FireStrength), ImmToMyDouble(12));
    }
}

CMDFUNC(cmdSyncAll)
{
    //set the body and gun rotation to match the current radar position
    RobotData->SystemVars[_gunrmn] = MyDouble_MOD(MyDouble_SUB(RobotData->SystemVars[_gunaim], RobotData->SystemVars[_radaraim]), ImmToMyDouble(360));
    RobotData->SystemVars[_bodyrmn] = MyDouble_MOD(MyDouble_SUB(RobotData->SystemVars[_bodyaim], RobotData->SystemVars[_radaraim]), ImmToMyDouble(360));
    RobotData->SystemVars[_rotating].SubNumber = 1;
    
    if(RobotData->Blocking)
        NEXT_STATE(WAIT_ON_ROTATION);
}

CMDFUNC(cmdSyncGun)
{
    //set the gun rotation to match the current radar position
    RobotData->SystemVars[_gunrmn] = MyDouble_MOD(MyDouble_SUB(RobotData->SystemVars[_gunaim], RobotData->SystemVars[_radaraim]), ImmToMyDouble(360));
    RobotData->SystemVars[_rotating].SubNumber = 1;
    
    if(RobotData->Blocking)
        NEXT_STATE(WAIT_ON_ROTATION);
}

CMDFUNC(cmdLockAll)
{
    //lock body/gun/radar rotation together
    if(!MyDouble_EQUAL(GETVAR(), MyDouble0))
        RobotData->RotationLock = ROTATE_LOCK_BODY | ROTATE_LOCK_RADAR;
    else
        RobotData->RotationLock = 0;
    if(!MyDouble_EQUAL(RobotData->SystemVars[_rotating], MyDouble0))
    {
        RobotData->SystemVars[_rotating] = MyDouble0;
        RobotData->SystemVars[_bodyrmn] = MyDouble0;
        RobotData->SystemVars[_gunrmn] = MyDouble0;
        RobotData->SystemVars[_radarrmn] = MyDouble0;
    }
}

CMDFUNC(cmdLockGun)
{
    //lock gun/radar rotation together
    if(!MyDouble_EQUAL(GETVAR(), MyDouble0))
        RobotData->RotationLock |= ROTATE_LOCK_RADAR;
    else
        RobotData->RotationLock &= ~ROTATE_LOCK_RADAR;
    
    //if we are rotating see if it is the gun or radar and stop them if so
    if(RobotData->SystemVars[_rotating].SubNumber)
    {
        if((!MyDouble_EQUAL(RobotData->SystemVars[_gunrmn], MyDouble0)) || !(MyDouble_EQUAL(RobotData->SystemVars[_radarrmn], MyDouble0)))
        {
            //if the body isn't rotating then turn off the rotate flag
            if(MyDouble_EQUAL(RobotData->SystemVars[_bodyrmn], MyDouble0))
                RobotData->SystemVars[_rotating] = MyDouble0;
            RobotData->SystemVars[_gunrmn] = MyDouble0;
            RobotData->SystemVars[_radarrmn] = MyDouble0;
        }
    }
}

CMDFUNC(cmdEndTurn)
{
    NEXT_STATE(WAITFOR_STATE);
}

CMDFUNC(cmdSuicide)
{
    NEXT_STATE(KILLED);
}

CMDFUNC(cmdBlocking)
{
    RobotData->Blocking = (int)GETVAR().SubNumber;
}

CMDFUNC(cmdStall)
{
    RobotData->Stalled = (int)GETVAR().SubNumber;
    NEXT_STATE(STALLED);
}

CMDFUNC(cmdIsEqual)
{
    MyDouble Var1;
    MyDouble Var2;
    MyDouble Var3;

    Var1 = GETVAR();
    Var2 = GETVAR();

    Var3.SubNumber = MyDouble_EQUAL(Var1 ,Var2);
    Var3.Fraction = 0;
    Var3.Signed = 0;
    RobotData->SystemVars[_result] = Var3;
}

CMDFUNC(cmdAdd)
{
    MyDouble Var1;
    MyDouble Var2;
    
    Var1 = GETVAR();
    Var2 = GETVAR();

    SETVAR(MyDouble_ADD(Var1, Var2));
}

CMDFUNC(cmdSub)
{
    MyDouble Var1;
    MyDouble Var2;
    
    Var1 = GETVAR();
    Var2 = GETVAR();
    
    SETVAR(MyDouble_SUB(Var1, Var2));
}

CMDFUNC(cmdDivide)
{
    MyDouble Var1;
    MyDouble Var2;
    
    Var1 = GETVAR();
    Var2 = GETVAR();
    
    SETVAR(MyDouble_DIV(Var1, Var2));
}

CMDFUNC(cmdMultiply)
{
    MyDouble Var1;
    MyDouble Var2;
    
    Var1 = GETVAR();
    Var2 = GETVAR();
    
    SETVAR(MyDouble_MUL(Var1, Var2));
}

CMDFUNC(cmdExponent)
{
    MyDouble Var1;
    MyDouble Var2;
    
    Var1 = GETVAR();
    Var2 = GETVAR();
    
    SETVAR(MyDouble_POW(Var1, Var2));
}

CMDFUNC(cmdModulus)
{
    MyDouble Var1;
    MyDouble Var2;
    
    Var1 = GETVAR();
    Var2 = GETVAR();
    
    SETVAR(MyDouble_MOD(Var1, Var2));
}

CMDFUNC(cmdCos)
{
    MyDouble Var1;
    
    Var1 = GETVAR();
    SETVAR(DoubleToMyDouble(cosf(MyDoubleToDouble(Var1))));
}

CMDFUNC(cmdSin)
{
    MyDouble Var1;
    
    Var1 = GETVAR();
    SETVAR(DoubleToMyDouble(sinf(MyDoubleToDouble(Var1))));
}

CMDFUNC(cmdTan)
{
    MyDouble Var1;
    
    Var1 = GETVAR();
    SETVAR(DoubleToMyDouble(tanf(MyDoubleToDouble(Var1))));
}

CMDFUNC(cmdACos)
{
    MyDouble Var1;
    
    Var1 = GETVAR();
    SETVAR(DoubleToMyDouble(acosf(MyDoubleToDouble(Var1))));
}

CMDFUNC(cmdASin)
{
    MyDouble Var1;
    
    Var1 = GETVAR();
    SETVAR(DoubleToMyDouble(asinf(MyDoubleToDouble(Var1))));
}

CMDFUNC(cmdATan)
{
    MyDouble Var1;
    
    Var1 = GETVAR();
    SETVAR(DoubleToMyDouble(atanf(MyDoubleToDouble(Var1))));
}

CMDFUNC(cmdATan2)
{
    MyDouble Var1;
    MyDouble Var2;

    Var1 = GETVAR();
    Var2 = GETVAR();
    SETVAR(DoubleToMyDouble(atan2f(MyDoubleToDouble(Var1), MyDoubleToDouble(Var2))));
}

CMDFUNC(cmdTruncate)
{
    MyDouble Var1;
    
    Var1 = GETVAR();
    RobotData->SystemVars[_result] = Var1;
    RobotData->SystemVars[_result].Fraction = 0;
}

CMDFUNC(cmdRound)
{
    MyDouble Var1;
    
    Var1 = GETVAR();
    RobotData->SystemVars[_result] = Var1;
    if(RobotData->SystemVars[_result].Fraction >= 50000)
    {
        if(RobotData->SystemVars[_result].Signed)
            RobotData->SystemVars[_result].SubNumber--;
        else
            RobotData->SystemVars[_result].SubNumber++;
    }
    RobotData->SystemVars[_result].Fraction = 0;
}

CMDFUNC(cmdSign)
{
    MyDouble Var;
    Var = GETVAR();
    if(MyDouble_EQUAL(Var, MyDouble0))
        RobotData->SystemVars[_result] = MyDouble0;
    else if(Var.Signed)
        RobotData->SystemVars[_result] = ImmToMyDouble(-1);
    else
        RobotData->SystemVars[_result] = ImmToMyDouble(1);
}

CMDFUNC(cmdMin)
{
    MyDouble Var1;
    MyDouble Var2;
    
    Var1 = GETVAR();
    Var2 = GETVAR();
    
    if(MyDouble_LESSTHAN_EQUAL(Var1, Var2))
        RobotData->SystemVars[_result] = Var1;
    else
        RobotData->SystemVars[_result] = Var2;
}

CMDFUNC(cmdMax)
{
    MyDouble Var1;
    MyDouble Var2;
    
    Var1 = GETVAR();
    Var2 = GETVAR();
    
    if(MyDouble_GREATERTHAN_EQUAL(Var1, Var2))
        RobotData->SystemVars[_result] = Var1;
    else
        RobotData->SystemVars[_result] = Var2;
}

CMDFUNC(cmdAbs)
{
    MyDouble Var1;
    
    Var1 = GETVAR();    
    RobotData->SystemVars[_result] = Var1;
    RobotData->SystemVars[_result].Signed = 0;
}

CMDFUNC(cmdGetRandom)
{
    int Rand = rand() & 0x7fff;
    RobotData->SystemVars[_result].SubNumber = Rand & 0x3fff;
    RobotData->SystemVars[_result].Fraction = rand() % 100000;
    RobotData->SystemVars[_result].Signed = (Rand >> 14) & 1;
}

CMDFUNC(cmdAnd)
{
    MyDouble Var1;
    MyDouble Var2;
    
    Var1 = GETVAR();
    Var2 = GETVAR();

    if(!(MyDouble_EQUAL(Var1, MyDouble0)) && !(MyDouble_EQUAL(Var2, MyDouble0)))
        SETVAR((ImmToMyDouble(-1)));
    else
        SETVAR(MyDouble0);
}

CMDFUNC(cmdOr)
{
    MyDouble Var1;
    MyDouble Var2;
    
    Var1 = GETVAR();
    Var2 = GETVAR();

    if(!(MyDouble_EQUAL(Var1, MyDouble0)) || !(MyDouble_EQUAL(Var2, MyDouble0)))
        SETVAR((ImmToMyDouble(-1)));
    else
        SETVAR(MyDouble0);
}

CMDFUNC(cmdEquals)
{
    MyDouble Var1;
    MyDouble Var2;
    
    Var1 = GETVAR();
    Var2 = GETVAR();

    if(MyDouble_EQUAL(Var1, Var2))
        SETVAR((ImmToMyDouble(-1)));
    else
        SETVAR(MyDouble0);
}

CMDFUNC(cmdLessThan)
{
    MyDouble Var1;
    MyDouble Var2;
    
    Var1 = GETVAR();
    Var2 = GETVAR();

    if(MyDouble_LESSTHAN(Var1, Var2))

        SETVAR((ImmToMyDouble(-1)));
    else
        SETVAR(MyDouble0);
}

CMDFUNC(cmdLessThanEqualTo)
{
    MyDouble Var1;
    MyDouble Var2;
    
    Var1 = GETVAR();
    Var2 = GETVAR();

    if(MyDouble_LESSTHAN_EQUAL(Var1, Var2))
        SETVAR((ImmToMyDouble(-1)));
    else
        SETVAR(MyDouble0);
}
CMDFUNC(cmdGreaterThan)
{
    MyDouble Var1;
    MyDouble Var2;
    
    Var1 = GETVAR();
    Var2 = GETVAR();
    if(MyDouble_GREATERTHAN(Var1, Var2))
        SETVAR((ImmToMyDouble(-1)));
    else
        SETVAR(MyDouble0);
}

CMDFUNC(cmdGreaterThanEqualTo)
{
    MyDouble Var1;
    MyDouble Var2;
    
    Var1 = GETVAR();
    Var2 = GETVAR();

    if(MyDouble_GREATERTHAN_EQUAL(Var1, Var2))
        SETVAR((ImmToMyDouble(-1)));
    else
        SETVAR(MyDouble0);
}

CMDFUNC(cmdNotEqual)
{
    MyDouble Var1;
    MyDouble Var2;
    
    Var1 = GETVAR();
    Var2 = GETVAR();

    if(!MyDouble_EQUAL(Var1, Var2))
        SETVAR((ImmToMyDouble(-1)));
    else
        SETVAR(MyDouble0);
}

CMDFUNC(cmdIf)
{
    //check and see if we should skip over the next block of commands
    MyDouble Var1;
    int SkipPos;
    
    //go short to int for sign extend
    SkipPos = Func->CmdPos + (int)*(short *)&Func->Cmds[Func->CmdPos];
    Func->CmdPos += 2;
    Var1 = GETVAR();
    
    //if false then set the skip position
    if(MyDouble_EQUAL(Var1, MyDouble0))
        Func->CmdPos = SkipPos;

    Func->CmdPos %= Func->CmdLen;
}

CMDFUNC(cmdJmp)
{
    //jump to a location
    int SkipCount;

    //get two bytes for how many bytes to jump, go from short to int to sign extend
    SkipCount = (int)*(short *)&Func->Cmds[Func->CmdPos];
    Func->CmdPos = (Func->CmdPos + SkipCount) % Func->CmdLen;
}

CMDFUNC(cmdRegAScan)
{
    int FuncID;
    int Priority;
    
    FuncID = GETFUNCID();
    Priority = ((MyDouble)GETVAR()).SubNumber;
    if(Priority <= 0)
        Priority = 1;
    else if(Priority > 9)
        Priority = 9;
    RobotData->Events[EVENT_AUTOSCAN]->Function = RobotData->Functions[FuncID];
    RobotData->Events[EVENT_AUTOSCAN]->Function->CmdPos = 0;
    RobotData->Events[EVENT_AUTOSCAN]->EventLevel = Priority;
}

CMDFUNC(cmdRegCldRobot)
{
    int FuncID;
    int Priority;
    
    FuncID = GETFUNCID();
    Priority = ((MyDouble)GETVAR()).SubNumber;
    if(Priority <= 0)
        Priority = 1;
    else if(Priority > 9)
        Priority = 9;
    RobotData->Events[EVENT_ROBOTCOLLISION]->Function = RobotData->Functions[FuncID];
    RobotData->Events[EVENT_ROBOTCOLLISION]->Function->CmdPos = 0;
    RobotData->Events[EVENT_ROBOTCOLLISION]->EventLevel = Priority;
}

CMDFUNC(cmdRegCldMissile)
{
    int FuncID;
    int Priority;
    
    FuncID = GETFUNCID();
    Priority = ((MyDouble)GETVAR()).SubNumber;
    if(Priority <= 0)
        Priority = 1;
    else if(Priority > 9)
        Priority = 9;
    RobotData->Events[EVENT_MISSILECOLLISION]->Function = RobotData->Functions[FuncID];
    RobotData->Events[EVENT_MISSILECOLLISION]->Function->CmdPos = 0;
    RobotData->Events[EVENT_MISSILECOLLISION]->EventLevel = Priority;
}

CMDFUNC(cmdRegCldWall)
{
    int FuncID;
    int Priority;
    
    FuncID = GETFUNCID();
    Priority = ((MyDouble)GETVAR()).SubNumber;
    if(Priority <= 0)
        Priority = 1;
    else if(Priority > 9)
        Priority = 9;
    RobotData->Events[EVENT_WALLCOLLISION]->Function = RobotData->Functions[FuncID];
    RobotData->Events[EVENT_WALLCOLLISION]->Function->CmdPos = 0;
    RobotData->Events[EVENT_WALLCOLLISION]->EventLevel = Priority;
}

CMDFUNC(cmdRegCore)
{
    int FuncID;
    
    FuncID = GETFUNCID();
    RobotData->Events[EVENT_CORE]->Function = RobotData->Functions[FuncID];
    RobotData->Events[EVENT_CORE]->Function->CmdPos = 0;
}

CMDFUNC(cmdRegDtcRobot)
{
    int FuncID;
    int Priority;
    
    FuncID = GETFUNCID();
    Priority = ((MyDouble)GETVAR()).SubNumber;
    if(Priority <= 0)
        Priority = 1;
    else if(Priority > 9)
        Priority = 9;
    RobotData->Events[EVENT_ROBOTDETECT]->Function = RobotData->Functions[FuncID];
    RobotData->Events[EVENT_ROBOTDETECT]->Function->CmdPos = 0;
    RobotData->Events[EVENT_ROBOTDETECT]->EventLevel = Priority;
}

CMDFUNC(cmdRegDtcWall)
{
    int FuncID;
    int Priority;
    
    FuncID = GETFUNCID();
    Priority = ((MyDouble)GETVAR()).SubNumber;
    if(Priority <= 0)
        Priority = 1;
    else if(Priority > 9)
        Priority = 9;
    RobotData->Events[EVENT_WALLDETECT]->Function = RobotData->Functions[FuncID];
    RobotData->Events[EVENT_WALLDETECT]->Function->CmdPos = 0;
    RobotData->Events[EVENT_WALLDETECT]->EventLevel = Priority;
}

CMDFUNC(cmdRegPing)
{
    int FuncID;
    int Priority;
    
    FuncID = GETFUNCID();
    Priority = ((MyDouble)GETVAR()).SubNumber;
    if(Priority <= 0)
        Priority = 1;
    else if(Priority > 9)
        Priority = 9;
    RobotData->Events[EVENT_PING]->Function = RobotData->Functions[FuncID];
    RobotData->Events[EVENT_PING]->Function->CmdPos = 0;
    RobotData->Events[EVENT_PING]->EventLevel = Priority;
}

CMDFUNC(cmdAScanEvents)
{
    int Enabled = ((MyDouble)GETVAR()).SubNumber;
    
    //if enabled then reset the function
    if(Enabled)
    {
        if(RobotData->Events[EVENT_AUTOSCAN]->Function)
            RobotData->Events[EVENT_AUTOSCAN]->Function->CmdPos = 0;
        RobotData->Events[EVENT_AUTOSCAN]->CurState = IDLE_STATE;
    }
    else
        RobotData->Events[EVENT_AUTOSCAN]->CurState = STATE_DISABLED;
}

CMDFUNC(cmdCldWallEvents)
{
    int Enabled = ((MyDouble)GETVAR()).SubNumber;
    
    //if enabled then reset the function
    if(Enabled)
    {
        if(RobotData->Events[EVENT_WALLCOLLISION]->Function)
            RobotData->Events[EVENT_WALLCOLLISION]->Function->CmdPos = 0;
        RobotData->Events[EVENT_WALLCOLLISION]->CurState = IDLE_STATE;
    }
    else
        RobotData->Events[EVENT_WALLCOLLISION]->CurState = STATE_DISABLED;
}

CMDFUNC(cmdCldRobotEvents)
{
    int Enabled = ((MyDouble)GETVAR()).SubNumber;
    
    //if enabled then reset the function
    if(Enabled)
    {
        if(RobotData->Events[EVENT_ROBOTCOLLISION]->Function)
            RobotData->Events[EVENT_ROBOTCOLLISION]->Function->CmdPos = 0;
        RobotData->Events[EVENT_ROBOTCOLLISION]->CurState = IDLE_STATE;
    }
    else
        RobotData->Events[EVENT_ROBOTCOLLISION]->CurState = STATE_DISABLED;
}

CMDFUNC(cmdCoreEvents)
{
    int Enabled = ((MyDouble)GETVAR()).SubNumber;
    
    //if enabled then reset the function
    if(Enabled)
    {
        if(RobotData->Events[EVENT_CORE]->Function)
            RobotData->Events[EVENT_CORE]->Function->CmdPos = 0;
        RobotData->Events[EVENT_CORE]->CurState = IDLE_STATE;
    }
    else
        RobotData->Events[EVENT_CORE]->CurState = STATE_DISABLED;
}

CMDFUNC(cmdDtcRobotEvents)
{
    int Enabled = ((MyDouble)GETVAR()).SubNumber;
    
    //if enabled then reset the function
    if(Enabled)
    {
        if(RobotData->Events[EVENT_ROBOTDETECT]->Function)
            RobotData->Events[EVENT_ROBOTDETECT]->Function->CmdPos = 0;
        RobotData->Events[EVENT_ROBOTDETECT]->CurState = IDLE_STATE;
    }
    else
        RobotData->Events[EVENT_ROBOTDETECT]->CurState = STATE_DISABLED;
}

CMDFUNC(cmdDtcWallEvents)
{
    int Enabled = ((MyDouble)GETVAR()).SubNumber;
    
    //if enabled then reset the function
    if(Enabled)
    {
        if(RobotData->Events[EVENT_WALLDETECT]->Function)
            RobotData->Events[EVENT_WALLDETECT]->Function->CmdPos = 0;
        RobotData->Events[EVENT_WALLDETECT]->CurState = IDLE_STATE;
    }
    else
        RobotData->Events[EVENT_WALLDETECT]->CurState = STATE_DISABLED;
}

CMDFUNC(cmdPingEvents)
{
    int Enabled = ((MyDouble)GETVAR()).SubNumber;
    
    //if enabled then reset the function
    if(Enabled)
    {
        if(RobotData->Events[EVENT_PING]->Function)
            RobotData->Events[EVENT_PING]->Function->CmdPos = 0;
        RobotData->Events[EVENT_PING]->CurState = IDLE_STATE;
    }
    else
        RobotData->Events[EVENT_PING]->CurState = STATE_DISABLED;
}

CMDFUNC(cmdCldMissileEvents)
{
    int Enabled = ((MyDouble)GETVAR()).SubNumber;
    
    //if enabled then reset the function
    if(Enabled)
    {
        if(RobotData->Events[EVENT_MISSILECOLLISION]->Function)
            RobotData->Events[EVENT_MISSILECOLLISION]->Function->CmdPos = 0;
        RobotData->Events[EVENT_MISSILECOLLISION]->CurState = IDLE_STATE;
    }
    else
        RobotData->Events[EVENT_MISSILECOLLISION]->CurState = STATE_DISABLED;
}

CMDFUNC(cmdWaitFor)
{
    int JumpPos;
    MyDouble Val;
    
    //get the jump offset incase the value we read isn't good
    JumpPos = *(short *)&Func->Cmds[Func->CmdPos];
    JumpPos = Func->CmdPos + JumpPos;
    
    //now advance ahead and get the register
    Func->CmdPos += 2;
    Val = GETVAR();
    
    //if the value isn't set then wait longer
    if(MyDouble_EQUAL(Val, MyDouble0))
    {
        Func->CmdPos = JumpPos % Func->CmdLen;
        NEXT_STATE(WAITFOR_STATE);
    }
    
    Func->CmdPos %= Func->CmdLen;
}

/*
CMDFUNC(cmdGoSub)
{
    
}

CMDFUNC(cmdReturn)
{
    
}
*/

CMDFUNC(cmdGetTime)
{
    clock_t RetTime;
    MyDouble MyTime;

    RetTime = clock();
    MyTime = DoubleToMyDouble(((double)RetTime) / CLOCKS_PER_SEC);
    RobotData->SystemVars[_result] = MyTime;
}

CMDFUNC(cmdContinue)
{
    RobotData->StopMotion = 0;
    RobotData->StopRotation = 0;
}

CMDFUNC(cmdPrint)
{
    int ParamCount;
    int StrLen;
    MyDouble Val;
    
    //dump out the robot name
    StrLen  = my_strlen(RobotData->Name);
    write(1, "debug: ", 7);
    write(1, RobotData->Name, StrLen);
    write(1, ": ", 2);
    
    ParamCount = (unsigned char)Func->Cmds[Func->CmdPos];
    Func->CmdPos += 1;
    
    while(ParamCount)
    {
        ParamCount--;
        
        //see if this is a string, otherwise get the variable
        if((unsigned char)Func->Cmds[Func->CmdPos] == PARAM_STRING)
        {
            StrLen = (unsigned char)Func->Cmds[Func->CmdPos+1];
            write(1, &Func->Cmds[Func->CmdPos+2], StrLen);
            Func->CmdPos += StrLen + 2;
        }
        else
        {
            Val = GETVAR();
            my_printf(" %f ", Val);
        }
    }
    write(1, "\n", 1);
    Func->CmdPos %= Func->CmdLen;
}

//do nothing function
CMDFUNC(cmdNOP)
{
}

CmdFuncPtr CmdList[] = {
    cmdAssignMath,
    cmdName,
    cmdAhead,
    cmdBack,
    cmdAccelTarget,
    cmdDecelTarget,
    cmdSpeedTarget,
    cmdBodyLeft,
    cmdBodyRight,
    cmdGunLeft,
    cmdGunRight,
    cmdRadarLeft,
    cmdRadarRight,
    cmdStop,
    cmdStopMove,
    cmdStopRotate,
    cmdContinue,
    cmdScan,
    cmdFire,
    cmdSyncAll,
    cmdSyncGun,    //20
    cmdLockAll,
    cmdLockGun,
    cmdEndTurn,
    cmdStall,
    cmdSuicide,
    cmdRegAScan,
    cmdRegCldRobot,
    cmdRegCldMissile,
    cmdRegCldWall,
    cmdRegCore,
    cmdRegDtcRobot,
    cmdRegDtcWall,
    cmdRegPing,
    cmdAScanEvents,
    cmdCldWallEvents,
    cmdCldRobotEvents,
    cmdCoreEvents,
    cmdDtcRobotEvents,
    cmdDtcWallEvents,
    cmdPingEvents,    //40
    cmdCldMissileEvents,
    cmdBlocking,
    cmdWaitFor,
//    cmdGoSub,
//    cmdReturn,
    cmdIf,
    cmdNOP,
    cmdNOP,
    cmdNOP,
    cmdNOP,
    cmdNOP,
    cmdNOP,
    cmdNOP,
    cmdAbs,
    cmdMax,
    cmdMin,
    cmdRound,
    cmdTruncate,
    cmdGetTime,
    cmdGetRandom,
    cmdIsEqual,
    cmdPrint,
    cmdAdd,
    cmdSub,
    cmdModulus,
    cmdMultiply,
    cmdDivide,
    cmdExponent,
    cmdAnd,
    cmdOr,
    cmdEquals,
    cmdLessThan,
    cmdLessThanEqualTo,
    cmdGreaterThan,
    cmdGreaterThanEqualTo,
    cmdNotEqual,
    cmdCos,
    cmdSin,
    cmdTan,
    cmdACos,
    cmdASin,
    cmdATan,
    cmdATan2,
    cmdJmp
};
