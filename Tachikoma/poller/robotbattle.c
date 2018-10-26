#include "robotbattle.h"
#include "cmds.h"
#include "parsecmds.h"

#include <math.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>

volatile struct MissileStruct *Missiles;
volatile struct StateStruct GlobalState;
volatile struct StateStruct *CurrentState;
volatile char *ErrorMsg;

//called from a function with it's own stack so tracking vars
//this way is perfectly fine
int ProcessCommand(RobotStruct *RobotData, StateStruct *State)
{
    int CurCmd;
    
//if(RobotData->Events[EVENT_ROBOTDETECT]->Function)
//    my_printf("robot: %s, state %s, line %d of %d\n", RobotData->Name, State->Function->Name, State->Function->CmdPos, State->Function->CmdLen);

    if(State->Function->CmdLen == 0)
        NEXT_STATE(IDLE_STATE);
    
	//move on to the next byte as some commands will change state
	//make sure we also wrap when we hit the end of the buffer
    CurCmd = State->Function->CmdPos;
    State->Function->CmdPos++;
    State->Function->CmdPos = State->Function->CmdPos % State->Function->CmdLen;
//if(RobotData->Events[EVENT_ROBOTDETECT]->Function)
//    my_printf("Cmd: (%d) %s\n", State->Function->Cmds[CurCmd], ParseCmds[State->Function->Cmds[CurCmd]].Name);
    CmdList[State->Function->Cmds[CurCmd]](RobotData, State->Function);

    if(State->Function->CmdPos)
        NEXT_STATE(NEXT_COMMAND);
    else
        NEXT_STATE(IDLE_STATE);
    return 0;
}

//function has it's own stack
void HandleState(RobotStruct *RobotData, int State)
{
    volatile int CycleCount = 0;
    volatile int Initialized = 0;
    
    //if we haven't initialized then initialize then swap over to the global state
    SAVE_EVENT_STATE(RobotData->Events[State], 1);
    if(!Initialized)
    {
        Initialized = 1;
        RobotData->Events[State]->CurState = IDLE_STATE;
        SWAP_TO_STATE(&GlobalState);
    }

    switch(CurrentState->CurState)
    {
        case STATE_DISABLED:
            //state is disabled. stop processing
            break;
            
        case IDLE_STATE:
            //if cycle count is 0 then continue on
            //otherwise set it to 0 and bail out as we need
            //to allow other processing to occur
            if(CycleCount)
            {
                CycleCount = 0;
                break;
            }

            //grab the trigger event data and fill it in if it exists, not everything uses a trigger
            if(RobotData->Events[State]->TriggerCount)
            {
                RobotData->Events[State]->TriggerCount--;
                if(State == EVENT_ROBOTDETECT)
                {
                    RobotData->SystemVars[_dtcbearing] = RobotData->Events[State]->Triggers[RobotData->Events[State]->TriggerCount].Bearing;
                    RobotData->SystemVars[_dtcheading] = RobotData->Events[State]->Triggers[RobotData->Events[State]->TriggerCount].Heading;
                    RobotData->SystemVars[_dtcdistance] = RobotData->Events[State]->Triggers[RobotData->Events[State]->TriggerCount].Distance;
                    RobotData->SystemVars[_dtcenergy] = RobotData->Events[State]->Triggers[RobotData->Events[State]->TriggerCount].Energy;
                    RobotData->SystemVars[_cldbearing] = MyDouble0;
                    RobotData->SystemVars[_cldheading] = MyDouble0;
                }
                else
                {
                    RobotData->SystemVars[_cldbearing] = RobotData->Events[State]->Triggers[RobotData->Events[State]->TriggerCount].Bearing;
                    RobotData->SystemVars[_cldheading] = RobotData->Events[State]->Triggers[RobotData->Events[State]->TriggerCount].Heading;
                    RobotData->SystemVars[_dtcbearing] = MyDouble0;
                    RobotData->SystemVars[_dtcheading] = MyDouble0;
                    RobotData->SystemVars[_dtcenergy] = MyDouble0;
                    RobotData->SystemVars[_dtcdistance] = MyDouble0;
                }
            }

        case NEXT_COMMAND:
            CycleCount++;
            ProcessCommand(RobotData, CurrentState);
            break;

        case WAITFOR_STATE:
            //if we've been processing then bail as we aren't done yet
            if(CycleCount)
            {
                CycleCount = 0;
                break;
            }
            CycleCount++;
            ProcessCommand(RobotData, CurrentState);
            break;

        case WAIT_ON_MOTION:
            if(RobotData->SystemVars[_moving].SubNumber)
                break;

            //if we aren't waiting for motion then go to the next command
            CycleCount = 0;
            NEXT_STATE(NEXT_COMMAND);

        case WAIT_ON_ROTATION:
            if(RobotData->SystemVars[_rotating].SubNumber)
                break;

            //if we aren't waiting for motion then go to the next command
            CycleCount = 0;
            NEXT_STATE(NEXT_COMMAND);

		case STALLED:
            //if we were stalled then fix ourselves
			if(RobotData->Stalled == 0)
				NEXT_STATE(NEXT_COMMAND);
			
			break;
        case KILLED:
            RobotData->Alive = 0;
            NEXT_STATE(STATE_DISABLED);

        case CMD_ERROR:
            write(1, ErrorMsg, strlen(ErrorMsg));
            write(1, "\n", 1);
            
        default:
            NEXT_STATE(KILLED);
    };

    //all done, signal next state to occur
    SWAP_TO_STATE(&GlobalState);
    return;
}

MyDouble CalcDistance(MyDouble X1, MyDouble Y1, MyDouble X2, MyDouble Y2)
{
    double x, y;
    x = MyDoubleToDouble(X1) - MyDoubleToDouble(X2);
    y = MyDoubleToDouble(Y1) - MyDoubleToDouble(Y2);
    x = (x*x) + (y*y);
    return DoubleToMyDouble(sqrt(x));
}

//determine if two lines cross each other
char CheckForCollision(MyDouble Line1Start_x, MyDouble Line1Start_y, MyDouble Line1End_x, MyDouble Line1End_y,
                       MyDouble Line2Start_x, MyDouble Line2Start_y, MyDouble Line2End_x, MyDouble Line2End_y,
                       MyDouble *Intersect_x, MyDouble *Intersect_y)
{
    double s1_x, s1_y, s2_x, s2_y;
    s1_x = MyDoubleToDouble(Line1End_x) - MyDoubleToDouble(Line1Start_x);
    s1_y = MyDoubleToDouble(Line1End_y) - MyDoubleToDouble(Line1Start_y);
    s2_x = MyDoubleToDouble(Line2End_x) - MyDoubleToDouble(Line2Start_x);
    s2_y = MyDoubleToDouble(Line2End_y) - MyDoubleToDouble(Line2Start_y);
    
    double s, t;
    s = (-s1_y * (MyDoubleToDouble(Line1Start_x) - MyDoubleToDouble(Line2Start_x)) + s1_x * (MyDoubleToDouble(Line1Start_y) - MyDoubleToDouble(Line2Start_y))) / (-s2_x * s1_y + s1_x * s2_y);
    t = ( s2_x * (MyDoubleToDouble(Line1Start_y) - MyDoubleToDouble(Line2Start_y)) - s2_y * (MyDoubleToDouble(Line1Start_x) - MyDoubleToDouble(Line2Start_x))) / (-s2_x * s1_y + s1_x * s2_y);

    //s = MyDouble_DIV(MyDouble_ADD(MyDouble_MUL(MyDouble_NEG(s1_y), MyDouble_SUB(Line1Start_x, Line2Start_x)), MyDouble_MUL(s1_x, MyDouble_SUB(Line1Start_y, Line2Start_y))), MyDouble_ADD(MyDouble_MUL(MyDouble_NEG(s2_x), s1_y), MyDouble_MUL(s1_x, s2_y)));
    //t = MyDouble_DIV(MyDouble_SUB(MyDouble_MUL(s2_x, MyDouble_SUB(Line1Start_y, Line2Start_y)), MyDouble_MUL(s2_y, MyDouble_SUB(Line1Start_x, Line2Start_x))), MyDouble_ADD(MyDouble_MUL(MyDouble_NEG(s2_x), s1_y), MyDouble_MUL(s1_x, s2_y)));
    if ((s >= 0.0) && (s <= 1.0) && (t >= 0.0) && (t <= 1.0))
    {
        // Collision detected
        if(Intersect_x)
            *Intersect_x = DoubleToMyDouble(MyDoubleToDouble(Line1Start_x) + (t * s1_x));
        if(Intersect_y)
            *Intersect_y = DoubleToMyDouble(MyDoubleToDouble(Line1Start_y) + (t * s1_y));
        return 1;
    }

    return 0; // No collision
}

void HandleSystemEvents(int RobotCount, RobotStruct **Robots, RobotStruct *SystemData)
{
    volatile int Initialized = 0;
    volatile StateStruct TempState;
    volatile MissileStruct *CurMissile;
    volatile MissileStruct *PrevMissile;
    volatile MissileStruct *TempMissile;
    volatile int RobotID;
    volatile MyDouble MoveAmount;
    volatile MyDouble EndXPos, EndYPos;
    volatile MyDouble EndOffset;
    volatile MyDouble DecelDistance;
    volatile int Temp;
    volatile MyDouble XHitPos, YHitPos;
    
    //if we haven't initialized then initialize then swap over to the global state
    SystemData->Events[EVENT_CORE] = (StateStruct *)malloc(sizeof(StateStruct));
    SAVE_EVENT_STATE(&TempState, 0);
    SAVE_EVENT_STATE(SystemData->Events[EVENT_CORE], 1);
    if(!Initialized)
    {
        Initialized = 1;
        SystemData->Initialized = 1;
        SystemData->Alive = 1;
        SystemData->Events[EVENT_CORE]->CurState = IDLE_STATE;
        SystemData->Events[EVENT_CORE]->Function = 0xffffffff;
        SWAP_TO_STATE(&GlobalState);
    }

    //work through all system things, missiles, robot motion, etc
    CurMissile = Missiles;
    RobotID = -1;

    UPDATE_EVENT_STATE(&TempState, 1);

    if(CurMissile)
    {
        //move the missile
        EndXPos = DoubleToMyDouble(sin(MyDoubleToDouble(CurMissile->Heading) * DEG_TO_RADIAN) * MISSILE_SPEED);
        EndYPos = DoubleToMyDouble(cos(MyDoubleToDouble(CurMissile->Heading) * DEG_TO_RADIAN) * MISSILE_SPEED);

        CurMissile->OldX = CurMissile->X;
        CurMissile->OldY = CurMissile->Y;
        CurMissile->X = MyDouble_ADD(CurMissile->X, EndXPos);
        CurMissile->Y = MyDouble_ADD(CurMissile->Y, EndYPos);

        //adjust the missile energy
        if(MyDouble_GREATERTHAN(CurMissile->Energy, ImmToMyDouble(4)))
        {
            CurMissile->Energy = MyDouble_SUB(CurMissile->Energy, ImmToMyDouble(1));
            if(MyDouble_LESSTHAN(CurMissile->Energy, ImmToMyDouble(4)))
                CurMissile->Energy = ImmToMyDouble(4);
        }
        
        //we don't bounds check yet as we need to see if we hit a robot first

        //next missile
        CurMissile = CurMissile->Next;
        SWAP_TO_STATE(&TempState);
    }

    //advance to the next robot
    RobotID++;
    if(RobotID < RobotCount)
    {
        //only process alive robots
        if(Robots[RobotID]->Alive)
        {
            //my_printf("System state: Robot %d\n", RobotID);
            
            //does the robot need to move?
            if(Robots[RobotID]->SystemVars[_moving].SubNumber && !Robots[RobotID]->StopMotion)
            {
                //move the robot
                
                //first, figure out if the velocity needs to be increased or decreased
                MoveAmount = MyDouble_ABS(Robots[RobotID]->SystemVars[_velocity]);

                if(!Robots[RobotID]->Decelerating)
                {
                    //we aren't decelerating yet, see if we need to accelerate
                    if(MyDouble_LESSTHAN(MoveAmount, Robots[RobotID]->SystemVars[_speedtarget]))
                    {
                        MoveAmount = MyDouble_ADD(MoveAmount, Robots[RobotID]->SystemVars[_accelltarget]);
                        if(MyDouble_GREATERTHAN(MoveAmount, Robots[RobotID]->SystemVars[_speedtarget]))
                            MoveAmount = Robots[RobotID]->SystemVars[_speedtarget];
                    }
                    
                    //if we allow the robot to increase speed once more, will we decel fast enough?
                    //calculate the next speed increase
                    if(MyDouble_LESSTHAN(MoveAmount, Robots[RobotID]->SystemVars[_speedtarget]))
                    {
                        EndXPos = MyDouble_ADD(MoveAmount, Robots[RobotID]->SystemVars[_accelltarget]);
                        if(MyDouble_GREATERTHAN(EndXPos, Robots[RobotID]->SystemVars[_speedtarget]))
                            EndXPos = Robots[RobotID]->SystemVars[_speedtarget];
                    }
                    else
                        EndXPos = MoveAmount;
                    
                    //see if it is time to turn on the decel flag
                    //calculate the distance needed to decel from the speed we would be running at next cycle
                    DecelDistance = MyDouble_DIV(EndXPos, Robots[RobotID]->SystemVars[_deceltarget]);
                    DecelDistance = MyDouble_DIV(MyDouble_MUL(MyDouble_MUL(Robots[RobotID]->SystemVars[_deceltarget], DecelDistance), MyDouble_ADD(DecelDistance, ImmToMyDouble(1))), ImmToMyDouble(2));
                    
                    //if not enough room to increase the speed again then indicate we will be decelerating
                    if(MyDouble_LESSTHAN_EQUAL(Robots[RobotID]->SystemVars[_movermn], DecelDistance))
                        Robots[RobotID]->Decelerating = 1;
                }
                else
                {
                    //keep decreasing the speed
                    MoveAmount = MyDouble_SUB(MoveAmount, Robots[RobotID]->SystemVars[_deceltarget]);
                }

                //make sure we aren't going to go too far
                if(MyDouble_GREATERTHAN(MoveAmount, Robots[RobotID]->SystemVars[_movermn]))
                    MoveAmount = Robots[RobotID]->SystemVars[_movermn];
                
                //calculate how far the robot will move in X and Y
                EndXPos = DoubleToMyDouble(sin(MyDoubleToDouble(Robots[RobotID]->SystemVars[_bodyaim]) * DEG_TO_RADIAN) * MyDoubleToDouble(MoveAmount));
                
                //if the robot is trying to go outside of the boundary then we need to calculate where we hit
                //both for damage and to keep the robot inside of the field
                Robots[RobotID]->OldX = Robots[RobotID]->SystemVars[_xpos];
                Robots[RobotID]->OldY = Robots[RobotID]->SystemVars[_ypos];
                Robots[RobotID]->SystemVars[_xpos] = MyDouble_ADD(Robots[RobotID]->SystemVars[_xpos], EndXPos);
                
                //see if we went outside of the X boundary
                EndOffset = MyDouble0;
                if(MyDouble_LESSTHAN_EQUAL(Robots[RobotID]->SystemVars[_xpos], FloatToMyDouble(ROBOT_SIZE / 2)))
                {
                    //calculate offset, position robot against the wall, x/y is center of robot
                    EndOffset = MyDouble_SUB(FloatToMyDouble(ROBOT_SIZE / 2), Robots[RobotID]->SystemVars[_xpos]);
                    Robots[RobotID]->SystemVars[_xpos] = DoubleToMyDouble((double)ROBOT_SIZE / 2.0);
                }
                else if(MyDouble_GREATERTHAN_EQUAL(Robots[RobotID]->SystemVars[_xpos], FloatToMyDouble(FIELD_SIZE - (ROBOT_SIZE / 2))))
                {
                    //put against the wall, x/y is center of robot
                    EndOffset = MyDouble_SUB(Robots[RobotID]->SystemVars[_xpos], FloatToMyDouble(FIELD_SIZE - (ROBOT_SIZE / 2.0)));
                    Robots[RobotID]->SystemVars[_xpos] = DoubleToMyDouble(FIELD_SIZE - (ROBOT_SIZE / 2.0));
                }

                if(!MyDouble_EQUAL(EndOffset, MyDouble0))
                {
                    //went outside, damage the robot
                    Robots[RobotID]->SystemVars[_energy] = MyDouble_SUB(Robots[RobotID]->SystemVars[_energy], ImmToMyDouble(1));
                    my_printf("WC %d\n", RobotID);

                    //calculate what our move amount should have been
                    EndXPos = MyDouble_SUB(EndXPos, EndOffset);
                    MoveAmount = MyDouble_DIV(EndXPos, DoubleToMyDouble(cos(MyDoubleToDouble(Robots[RobotID]->SystemVars[_bodyaim]) * DEG_TO_RADIAN)));
                    
                    //if the robot is handling collisions then add this collision
                    if(Robots[RobotID]->Events[EVENT_WALLCOLLISION]->Function)
                    {
                        //robot bearing is easy, if going forwards then 0 otherwise -180
                        if(MyDouble_LESSTHAN(Robots[RobotID]->SystemVars[_velocity], MyDouble0))
                        {
                            Robots[RobotID]->Events[EVENT_WALLCOLLISION]->Triggers[Robots[RobotID]->Events[EVENT_WALLCOLLISION]->TriggerCount].Bearing = ImmToMyDouble(-180);
                        }
                        else
                        {
                            Robots[RobotID]->Events[EVENT_WALLCOLLISION]->Triggers[Robots[RobotID]->Events[EVENT_WALLCOLLISION]->TriggerCount].Bearing = MyDouble0;
                        }
                        Robots[RobotID]->Events[EVENT_WALLCOLLISION]->Triggers[Robots[RobotID]->Events[EVENT_WALLCOLLISION]->TriggerCount].Heading = Robots[RobotID]->SystemVars[_bodyaim];
                        Robots[RobotID]->Events[EVENT_WALLCOLLISION]->TriggerCount++;
                    }
                }

                //now calculate Y as move amount may have changed
                EndYPos = MyDouble_MUL(DoubleToMyDouble(cos(MyDoubleToDouble(Robots[RobotID]->SystemVars[_bodyaim]) * DEG_TO_RADIAN)), MoveAmount);
                Robots[RobotID]->SystemVars[_ypos] = MyDouble_ADD(Robots[RobotID]->SystemVars[_ypos], EndYPos);
                EndOffset = MyDouble0;
                if(MyDouble_LESSTHAN_EQUAL(Robots[RobotID]->SystemVars[_ypos], FloatToMyDouble((ROBOT_SIZE / 2))))
                {
                    //calculate offset, position robot against the wall, x/y is center of robot
                    EndOffset = MyDouble_SUB(FloatToMyDouble(ROBOT_SIZE / 2), Robots[RobotID]->SystemVars[_ypos]);
                    Robots[RobotID]->SystemVars[_ypos] = FloatToMyDouble(ROBOT_SIZE / 2);
                }
                else if(MyDouble_GREATERTHAN_EQUAL(Robots[RobotID]->SystemVars[_ypos], FloatToMyDouble(FIELD_SIZE - (ROBOT_SIZE / 2))))
                {
                    //put against the wall, x/y is center of robot
                    EndOffset = MyDouble_SUB(Robots[RobotID]->SystemVars[_ypos], FloatToMyDouble(FIELD_SIZE - (ROBOT_SIZE / 2)));
                    Robots[RobotID]->SystemVars[_ypos] = FloatToMyDouble(FIELD_SIZE - (ROBOT_SIZE / 2));
                }
                
                if(!MyDouble_EQUAL(EndOffset, MyDouble0))
                {
                    //went outside, damage the robot, in theory you could hit the corner and get 2x damage
                    Robots[RobotID]->SystemVars[_energy] = MyDouble_SUB(Robots[RobotID]->SystemVars[_energy], ImmToMyDouble(1));
                    my_printf("WC %d\n", RobotID);

                    //calculate what our move amount should have been, we don't need to adjust _ypos as
                    //the if statement above does it for us
                    EndYPos = MyDouble_SUB(EndYPos, EndOffset);
                    MoveAmount = MyDouble_DIV(EndYPos, DoubleToMyDouble(sin(MyDoubleToDouble(Robots[RobotID]->SystemVars[_bodyaim]) * DEG_TO_RADIAN)));
                    
                    //adjust X accordingly, worst we can do is back it up further to be where it should be
                    //(x*x) + (y*y) = (z*z)
                    //z = move amount
                    //x = sqrt((z*z) - (y*y))
                    EndOffset = DoubleToMyDouble(sqrt(MyDoubleToDouble(MyDouble_SUB(MyDouble_MUL(MoveAmount, MoveAmount), MyDouble_MUL(EndYPos, EndYPos)))));
                    if(MyDouble_LESSTHAN(EndYPos, MyDouble0))
                        EndOffset = MyDouble_NEG(EndOffset);
                    Robots[RobotID]->SystemVars[_xpos] = MyDouble_ADD(Robots[RobotID]->SystemVars[_xpos], EndOffset);
                    
                    //if the robot is handling collisions then add this collision
                    if(Robots[RobotID]->Events[EVENT_WALLCOLLISION]->Function)
                    {
                        //robot bearing is easy, if going forwards then 0 otherwise -180
                        if(Robots[RobotID]->SystemVars[_velocity].Signed)
                        {
                            Robots[RobotID]->Events[EVENT_WALLCOLLISION]->Triggers[Robots[RobotID]->Events[EVENT_WALLCOLLISION]->TriggerCount].Bearing = ImmToMyDouble(-180);
                        }
                        else
                        {
                            Robots[RobotID]->Events[EVENT_WALLCOLLISION]->Triggers[Robots[RobotID]->Events[EVENT_WALLCOLLISION]->TriggerCount].Bearing = MyDouble0;
                        }
                        Robots[RobotID]->Events[EVENT_WALLCOLLISION]->Triggers[Robots[RobotID]->Events[EVENT_WALLCOLLISION]->TriggerCount].Heading = Robots[RobotID]->SystemVars[_bodyaim];
                        Robots[RobotID]->Events[EVENT_WALLCOLLISION]->TriggerCount++;
                    }
                }

                //update our velocity
                if(Robots[RobotID]->SystemVars[_velocity].Signed < 0)
                    Robots[RobotID]->SystemVars[_velocity] = MyDouble_NEG(MoveAmount);
                else
                    Robots[RobotID]->SystemVars[_velocity] = MoveAmount;
                
                //update our move left
                Robots[RobotID]->SystemVars[_movermn] = MyDouble_SUB(Robots[RobotID]->SystemVars[_movermn], MoveAmount);
                if(MyDouble_EQUAL(Robots[RobotID]->SystemVars[_movermn], MyDouble0))
                {
                    Robots[RobotID]->Decelerating = 0;
                    Robots[RobotID]->SystemVars[_moving] = MyDouble0;
                    Robots[RobotID]->SystemVars[_velocity] = MyDouble0;
                }
                
                my_printf("R %d %f %f %f %f\n", RobotID, Robots[RobotID]->SystemVars[_energy], Robots[RobotID]->SystemVars[_xpos], Robots[RobotID]->SystemVars[_ypos], Robots[RobotID]->SystemVars[_movermn]);
            }
	    else if((Robots[RobotID]->SystemVars[_moving].SubNumber == 0) && (!MyDouble_EQUAL(Robots[RobotID]->OldX, Robots[RobotID]->SystemVars[_xpos]) || !MyDouble_EQUAL(Robots[RobotID]->OldY, Robots[RobotID]->SystemVars[_ypos])))
            {
                //update OldX and OldY so collision is good
                Robots[RobotID]->OldX = Robots[RobotID]->SystemVars[_xpos];
                Robots[RobotID]->OldY = Robots[RobotID]->SystemVars[_ypos];
            }

            if(Robots[RobotID]->SystemVars[_rotating].SubNumber && !Robots[RobotID]->StopRotation)
            {
                //robot is in the middle of rotating, see what needs to rotate
                if(Robots[RobotID]->RotationLock & ROTATE_LOCK_BODY)
                {
                    //rotate everything together
                    if(Robots[RobotID]->SystemVars[_bodyrmn].Signed)
                    {
                        EndOffset = ImmToMyDouble(-5);
                        
                        //if we are finishing up the rotation then indicate as such
                        if(MyDouble_GREATERTHAN_EQUAL(Robots[RobotID]->SystemVars[_bodyrmn], ImmToMyDouble(-5)))
                            EndOffset = Robots[RobotID]->SystemVars[_bodyrmn];
                    }
                    else
                    {
                        EndOffset = ImmToMyDouble(5);
                        
                        //if we are finishing up the rotation then indicate as such
                        if(MyDouble_LESSTHAN_EQUAL(Robots[RobotID]->SystemVars[_bodyrmn], ImmToMyDouble(5)))
                            EndOffset = Robots[RobotID]->SystemVars[_bodyrmn];
                    }
                    
                    //adjust all 3 rotations
                    Robots[RobotID]->SystemVars[_bodyaim] = MyDouble_ADD(Robots[RobotID]->SystemVars[_bodyaim], EndOffset);
                    Robots[RobotID]->SystemVars[_bodyrmn] = MyDouble_SUB(Robots[RobotID]->SystemVars[_bodyrmn], EndOffset);
                    
                    Robots[RobotID]->SystemVars[_gunaim] = MyDouble_ADD(Robots[RobotID]->SystemVars[_gunaim], EndOffset);
                    Robots[RobotID]->SystemVars[_gunrmn] = MyDouble_SUB(Robots[RobotID]->SystemVars[_gunrmn], EndOffset);

                    Robots[RobotID]->SystemVars[_radaraim] = MyDouble_ADD(Robots[RobotID]->SystemVars[_radaraim], EndOffset);
                    Robots[RobotID]->SystemVars[_radarrmn] = MyDouble_SUB(Robots[RobotID]->SystemVars[_radarrmn], EndOffset);
                }
                else if(Robots[RobotID]->RotationLock & ROTATE_LOCK_RADAR)
                {
                    if(!MyDouble_EQUAL(Robots[RobotID]->SystemVars[_bodyrmn], MyDouble0))
                    {
                        //handle body seperately
                        if(Robots[RobotID]->SystemVars[_bodyrmn].Signed)
                        {
                            EndOffset = ImmToMyDouble(-5);
                            
                            //if we are finishing up the rotation then indicate as such
                            if(MyDouble_GREATERTHAN_EQUAL(Robots[RobotID]->SystemVars[_bodyrmn], ImmToMyDouble(-5)))
                                EndOffset = Robots[RobotID]->SystemVars[_bodyrmn];
                        }
                        else
                        {
                            EndOffset = ImmToMyDouble(5);
                            
                            //if we are finishing up the rotation then indicate as such
                            if(MyDouble_LESSTHAN_EQUAL(Robots[RobotID]->SystemVars[_bodyrmn], ImmToMyDouble(5)))
                                EndOffset = Robots[RobotID]->SystemVars[_bodyrmn];
                        }
                        
                        Robots[RobotID]->SystemVars[_bodyaim] = MyDouble_ADD(Robots[RobotID]->SystemVars[_bodyaim], EndOffset);
                        Robots[RobotID]->SystemVars[_bodyrmn] = MyDouble_SUB(Robots[RobotID]->SystemVars[_bodyrmn], EndOffset);
                    }
                    
                    if(!MyDouble_EQUAL(Robots[RobotID]->SystemVars[_gunrmn], MyDouble0))
                    {
                        //rotate everything together
                        if(Robots[RobotID]->SystemVars[_gunrmn].Signed)
                        {
                            EndOffset = ImmToMyDouble(-10);
                            
                            //if we are finishing up the rotation then indicate as such
                            if(MyDouble_GREATERTHAN_EQUAL(Robots[RobotID]->SystemVars[_gunrmn], ImmToMyDouble(-10)))
                                EndOffset = Robots[RobotID]->SystemVars[_gunrmn];
                        }
                        else
                        {
                            EndOffset = ImmToMyDouble(10);
                            
                            //if we are finishing up the rotation then indicate as such
                            if(MyDouble_LESSTHAN_EQUAL(Robots[RobotID]->SystemVars[_gunrmn], ImmToMyDouble(10)))
                                EndOffset = Robots[RobotID]->SystemVars[_gunrmn];
                        }
                        
                        //adjust both gun and radar
                        Robots[RobotID]->SystemVars[_gunaim] = MyDouble_ADD(Robots[RobotID]->SystemVars[_gunaim], EndOffset);
                        Robots[RobotID]->SystemVars[_gunrmn] = MyDouble_SUB(Robots[RobotID]->SystemVars[_gunrmn], EndOffset);
                        Robots[RobotID]->SystemVars[_radaraim] = MyDouble_ADD(Robots[RobotID]->SystemVars[_radaraim], EndOffset);
                        Robots[RobotID]->SystemVars[_radarrmn] = MyDouble_SUB(Robots[RobotID]->SystemVars[_radarrmn], EndOffset);
                    }
                }
                else
                {
                    //no locking, all individual
                    if(!MyDouble_EQUAL(Robots[RobotID]->SystemVars[_bodyrmn], MyDouble0))
                    {
                        //handle body
                        if(Robots[RobotID]->SystemVars[_bodyrmn].Signed)
                        {
                            EndOffset = ImmToMyDouble(-5);
                            
                            //if we are finishing up the rotation then indicate as such
                            if(MyDouble_GREATERTHAN_EQUAL(Robots[RobotID]->SystemVars[_bodyrmn], ImmToMyDouble(-5)))
                                EndOffset = Robots[RobotID]->SystemVars[_bodyrmn];
                        }
                        else
                        {
                            EndOffset = ImmToMyDouble(5);
                            
                            //if we are finishing up the rotation then indicate as such
                            if(MyDouble_LESSTHAN_EQUAL(Robots[RobotID]->SystemVars[_bodyrmn], ImmToMyDouble(5)))
                                EndOffset = Robots[RobotID]->SystemVars[_bodyrmn];
                        }

                        Robots[RobotID]->SystemVars[_bodyaim] = MyDouble_ADD(Robots[RobotID]->SystemVars[_bodyaim], EndOffset);
                        Robots[RobotID]->SystemVars[_bodyrmn] = MyDouble_SUB(Robots[RobotID]->SystemVars[_bodyrmn], EndOffset);
                    }

                    if(!MyDouble_EQUAL(Robots[RobotID]->SystemVars[_gunrmn], MyDouble0))
                    {
                        //handle gun
                        if(Robots[RobotID]->SystemVars[_gunrmn].Signed)
                        {
                            EndOffset = ImmToMyDouble(-10);
                            
                            //if we are finishing up the rotation then indicate as such
                            if(MyDouble_GREATERTHAN_EQUAL(Robots[RobotID]->SystemVars[_gunrmn], ImmToMyDouble(-10)))
                                EndOffset = Robots[RobotID]->SystemVars[_gunrmn];
                        }
                        else
                        {
                            EndOffset = ImmToMyDouble(10);
                            
                            //if we are finishing up the rotation then indicate as such
                            if(MyDouble_LESSTHAN_EQUAL(Robots[RobotID]->SystemVars[_gunrmn], ImmToMyDouble(10)))
                                EndOffset = Robots[RobotID]->SystemVars[_gunrmn];
                        }
                        Robots[RobotID]->SystemVars[_gunaim] = MyDouble_ADD(Robots[RobotID]->SystemVars[_gunaim], EndOffset);
                        Robots[RobotID]->SystemVars[_gunrmn] = MyDouble_SUB(Robots[RobotID]->SystemVars[_gunrmn], EndOffset);
                    }
                    
                    if(!MyDouble_EQUAL(Robots[RobotID]->SystemVars[_radarrmn], MyDouble0))
                    {
                        //handle radar
                        if(Robots[RobotID]->SystemVars[_radarrmn].Signed)
                        {
                            EndOffset = ImmToMyDouble(-15);
                            
                            //if we are finishing up the rotation then indicate as such
                            if(MyDouble_GREATERTHAN_EQUAL(Robots[RobotID]->SystemVars[_radarrmn], ImmToMyDouble(-15)))
                                EndOffset = Robots[RobotID]->SystemVars[_radarrmn];
                        }
                        else
                        {
                            EndOffset = ImmToMyDouble(15);
                            
                            //if we are finishing up the rotation then indicate as such
                            if(MyDouble_LESSTHAN_EQUAL(Robots[RobotID]->SystemVars[_radarrmn], ImmToMyDouble(15)))
                                EndOffset = Robots[RobotID]->SystemVars[_radarrmn];
                        }
                        Robots[RobotID]->SystemVars[_radaraim] = MyDouble_ADD(Robots[RobotID]->SystemVars[_radaraim], EndOffset);
                        Robots[RobotID]->SystemVars[_radarrmn] = MyDouble_SUB(Robots[RobotID]->SystemVars[_radarrmn], EndOffset);
                    }
                }

                Robots[RobotID]->SystemVars[_bodyaim] = MyDouble_MOD(Robots[RobotID]->SystemVars[_bodyaim], ImmToMyDouble(360));
                if(Robots[RobotID]->SystemVars[_bodyaim].Signed)
                    Robots[RobotID]->SystemVars[_bodyaim] = MyDouble_ADD(Robots[RobotID]->SystemVars[_bodyaim], ImmToMyDouble(360));
                
                Robots[RobotID]->SystemVars[_gunaim] = MyDouble_MOD(Robots[RobotID]->SystemVars[_gunaim], ImmToMyDouble(360));
                if(Robots[RobotID]->SystemVars[_gunaim].Signed)
                    Robots[RobotID]->SystemVars[_gunaim] = MyDouble_ADD(Robots[RobotID]->SystemVars[_gunaim], ImmToMyDouble(360));

                Robots[RobotID]->SystemVars[_radaraim] = MyDouble_MOD(Robots[RobotID]->SystemVars[_radaraim], ImmToMyDouble(360));
                if(Robots[RobotID]->SystemVars[_radaraim].Signed)
                    Robots[RobotID]->SystemVars[_radaraim] = MyDouble_ADD(Robots[RobotID]->SystemVars[_radaraim], ImmToMyDouble(360));

                //if a spot is = 0.0 then it is done
                Temp = 0;
                if(MyDouble_EQUAL(MyDouble_ABS(Robots[RobotID]->SystemVars[_bodyrmn]), MyDouble0))
                {
                    Robots[RobotID]->SystemVars[_bodyrmn] = MyDouble0;
                    Temp++;
                }
                
                if(MyDouble_EQUAL(MyDouble_ABS(Robots[RobotID]->SystemVars[_gunrmn]), MyDouble0))
                {
                    Robots[RobotID]->SystemVars[_gunrmn] = MyDouble0;
                    Temp++;
                }
                
                if(MyDouble_EQUAL(MyDouble_ABS(Robots[RobotID]->SystemVars[_radarrmn]), MyDouble0))
                {
                    Robots[RobotID]->SystemVars[_radarrmn] = MyDouble0;
                    Temp++;
                }

                //if nothing is rotating then stop the indicator
                if(Temp == 3)
                    Robots[RobotID]->SystemVars[_rotating] = MyDouble0;
            }
            
            //if the gun is hot then start cooling it, 1 unit per cycle
            if(!MyDouble_EQUAL(Robots[RobotID]->SystemVars[_gunheat], MyDouble0))
            {
                Robots[RobotID]->SystemVars[_gunheat] = MyDouble_SUB(Robots[RobotID]->SystemVars[_gunheat], ImmToMyDouble(1));
                if(MyDouble_LESSTHAN(Robots[RobotID]->SystemVars[_gunheat], MyDouble0))
                    Robots[RobotID]->SystemVars[_gunheat] = MyDouble0;
            }
        }

        //verify the robot didn't kill itself
        if((Robots[RobotID]->SystemVars[_energy].Signed) || MyDouble_EQUAL(Robots[RobotID]->SystemVars[_energy], MyDouble0))
            Robots[RobotID]->Alive = 0;

        //next robot
        SWAP_TO_STATE(&TempState);
    }
    
    //went through all robots, resync and setup position checks for everything
    CurMissile = Missiles;
    PrevMissile = 0;
    TempMissile = 0;
    RobotID = 0;
    
    UPDATE_EVENT_STATE(&TempState, 1);

    //check the current missile for any collisions against this robot, if any then remove it
    int RobotHit = 0;

    if(Robots[RobotID]->Alive && CurMissile)
    {
        MyDouble X1, Y1, X2, Y2;
	X1 = Robots[RobotID]->OldX;
	Y1 = Robots[RobotID]->OldY;
	X2 = Robots[RobotID]->SystemVars[_xpos];
	Y2 = Robots[RobotID]->SystemVars[_ypos];

	//look for a collision by drawing a line from two opposite corners of the robot from beginning to end of motion
        X1 = MyDouble_SUB(X1, FloatToMyDouble(ROBOT_SIZE / 2));
        Y1 = MyDouble_SUB(Y1, FloatToMyDouble(ROBOT_SIZE / 2));
        X2 = MyDouble_ADD(X2, FloatToMyDouble(ROBOT_SIZE / 2));
        Y2 = MyDouble_ADD(Y2, FloatToMyDouble(ROBOT_SIZE / 2));

        //cycle through all missiles looking for a collision across all robots
        if(CheckForCollision(X1, Y1, X2, Y2, CurMissile->OldX, CurMissile->OldY, CurMissile->X, CurMissile->Y, &XHitPos, &YHitPos))
        {
            //hit someone, damage the robot
            Robots[RobotID]->SystemVars[_energy] = MyDouble_SUB(Robots[RobotID]->SystemVars[_energy],  MyDouble_ADD(CurMissile->Energy, ImmToMyDouble(1)));     //deduct 1 for the collision itself
            if(MyDouble_LESSTHAN_EQUAL(Robots[RobotID]->SystemVars[_energy], MyDouble0))
                Robots[RobotID]->Alive = 0;

            RobotHit = 1;
            my_printf("MC %d %f %f %f\n", RobotID, Robots[RobotID]->SystemVars[_energy], XHitPos, YHitPos);

        //my_printf("Robot_XStart =  %f\nRobot_YStart = %f\nRobot_XEnd = %f\nRobot_YEnd = %f\n", X1, Y1, X2, Y2);
        //my_printf("Missile_XStart =  %f\nMissile_YStart = %f\nMissile_XEnd = %f\nMissile_YEnd = %f\n", CurMissile->OldX, CurMissile->OldY, CurMissile->X, CurMissile->Y);

            //if the robot is handling collisions then add this collision
            if(Robots[RobotID]->Events[EVENT_MISSILECOLLISION] && Robots[RobotID]->Events[EVENT_MISSILECOLLISION]->Function)
            {
                //calculate the bearing
                double x, y;
                x = MyDoubleToDouble(MyDouble_SUB(CurMissile->X, Robots[RobotID]->SystemVars[_xpos]));
                y = MyDoubleToDouble(MyDouble_SUB(CurMissile->Y, Robots[RobotID]->SystemVars[_ypos]));
                x = (atan2(x, y) * RADIAN_TO_DEG);
                if(x < 0.0)
                    x = 360.0 + x;

                //adjust for angle of the body
                x = x - MyDoubleToDouble(Robots[RobotID]->SystemVars[_bodyaim]);

                //now convert back to -180/+180
                if(x > 180.0)
                    x -= 360.0;
                else if(x < -180.0)
                    x += 360.0;

                //setup the heading and bearing
                Robots[RobotID]->Events[EVENT_MISSILECOLLISION]->Triggers[Robots[RobotID]->Events[EVENT_MISSILECOLLISION]->TriggerCount].Heading = CurMissile->Heading;
                Robots[RobotID]->Events[EVENT_MISSILECOLLISION]->Triggers[Robots[RobotID]->Events[EVENT_MISSILECOLLISION]->TriggerCount].Bearing = DoubleToMyDouble(x);
                Robots[RobotID]->Events[EVENT_MISSILECOLLISION]->TriggerCount++;
            }

            //now delete the missile
            if(PrevMissile)
                PrevMissile->Next = CurMissile->Next;
            else
                Missiles = CurMissile->Next;
            
            TempMissile = CurMissile;
            
            //advance CurMissile to the end so we don't cause any other missiles to die due to hitting a ghost
            if(!Robots[RobotID]->Alive)
            {
                while(CurMissile->Next)
                    CurMissile = CurMissile->Next;
            }
        }

        //check a cross section of the robot for collision
        if(!RobotHit)
        {
            X1 = Robots[RobotID]->OldX;
	    Y1 = Robots[RobotID]->OldY;
	    X2 = Robots[RobotID]->SystemVars[_xpos];
	    Y2 = Robots[RobotID]->SystemVars[_ypos];

	    //no collision so draw a line from the opposite corners of the robot
            X1 = MyDouble_ADD(X1, FloatToMyDouble(ROBOT_SIZE / 2));
            Y1 = MyDouble_ADD(Y1, FloatToMyDouble(ROBOT_SIZE / 2));
            X2 = MyDouble_SUB(X2, FloatToMyDouble(ROBOT_SIZE / 2));
            Y2 = MyDouble_SUB(Y2, FloatToMyDouble(ROBOT_SIZE / 2));

            //cycle through all missiles looking for a collision across all robots
            if(CheckForCollision(X1, Y1, X2, Y2, CurMissile->OldX, CurMissile->OldY, CurMissile->X, CurMissile->Y, &XHitPos, &YHitPos))
            {
                //hit someone, damage the robot
                Robots[RobotID]->SystemVars[_energy] = MyDouble_SUB(Robots[RobotID]->SystemVars[_energy],  MyDouble_ADD(CurMissile->Energy, ImmToMyDouble(1)));          //deduct 1 for the collision itself
                if(MyDouble_LESSTHAN_EQUAL(Robots[RobotID]->SystemVars[_energy], MyDouble0))
                    Robots[RobotID]->Alive = 0;

                my_printf("MC %d %f %f %f\n", RobotID, Robots[RobotID]->SystemVars[_energy], XHitPos, YHitPos);

                //if the robot is handling collisions then add this collision
                if(Robots[RobotID]->Events[EVENT_MISSILECOLLISION] && Robots[RobotID]->Events[EVENT_MISSILECOLLISION]->Function)
                {
                    //calculate the bearing
                    double x, y;
                    x = MyDoubleToDouble(MyDouble_SUB(CurMissile->X, Robots[RobotID]->SystemVars[_xpos]));
                    y = MyDoubleToDouble(MyDouble_SUB(CurMissile->Y, Robots[RobotID]->SystemVars[_ypos]));
                    x = (atan2(x, y) * RADIAN_TO_DEG);
                    if(x < 0.0)
                        x = 360.0 + x;

                    //adjust for angle of the body
                    x = x - MyDoubleToDouble(Robots[RobotID]->SystemVars[_bodyaim]);

                    //now convert back to -180/+180
                    if(x > 180.0)
                        x -= 360.0;
                    else if(x < -180.0)
                        x += 360.0;

                    //setup the heading and bearing
                    Robots[RobotID]->Events[EVENT_MISSILECOLLISION]->Triggers[Robots[RobotID]->Events[EVENT_MISSILECOLLISION]->TriggerCount].Heading = CurMissile->Heading;
                    Robots[RobotID]->Events[EVENT_MISSILECOLLISION]->Triggers[Robots[RobotID]->Events[EVENT_MISSILECOLLISION]->TriggerCount].Bearing = DoubleToMyDouble(x);
                    Robots[RobotID]->Events[EVENT_MISSILECOLLISION]->TriggerCount++;
                }

                //now delete the missile
                if(PrevMissile)
                    PrevMissile->Next = CurMissile->Next;
                else
                    Missiles = CurMissile->Next;
            
                TempMissile = CurMissile;
            
                //advance CurMissile to the end so we don't cause any other missiles to die due to hitting a ghost
                if(!Robots[RobotID]->Alive)
                {
                    while(CurMissile->Next)
                        CurMissile = CurMissile->Next;
                }
            }
        }

	//if the missle is outside of the arena then kill it
	if(!TempMissile && (MyDouble_LESSTHAN(CurMissile->X, MyDouble0) || MyDouble_LESSTHAN(CurMissile->Y, MyDouble0) || MyDouble_GREATERTHAN(CurMissile->X, ImmToMyDouble(FIELD_SIZE)) || MyDouble_GREATERTHAN(CurMissile->Y, ImmToMyDouble(FIELD_SIZE))))
	{
		//destroy the missile
		if(PrevMissile)
			PrevMissile->Next = CurMissile->Next;
		else
			Missiles = CurMissile->Next;

		TempMissile = CurMissile;
	}
        
        //next missile
        if(TempMissile)
        {
	    //temp got set, wipe it out
            free(TempMissile);
            TempMissile = 0;
        }
        else
            PrevMissile = CurMissile;	//update previous only when we don't delete a missile

        CurMissile = CurMissile->Next;
        SWAP_TO_STATE(&TempState);
    }

    //move on to the next robot
    RobotID++;
    if(RobotID < RobotCount)
    {
        CurMissile = Missiles;
        PrevMissile = 0;
        TempMissile = 0;
        SWAP_TO_STATE(&TempState);
    }

    //cycle through all of the missiles that are left and print them out
    CurMissile = Missiles;
    UPDATE_EVENT_STATE(&TempState, 1);
   
    if(CurMissile)
    {
        my_printf("M %f %f %f\n", CurMissile->Energy, CurMissile->X, CurMissile->Y);
        CurMissile = CurMissile->Next;
        SWAP_TO_STATE(&TempState);
    }

    //now setup and cycle through all robots looking for robot to robot collision
    RobotID = 0;
    Temp = 1;
    volatile MyDouble R1_X1, R1_X2, R1_Y1, R1_Y2;

    R1_X1 = Robots[RobotID]->SystemVars[_xpos];
    R1_X2 = Robots[RobotID]->SystemVars[_xpos];
    R1_Y1 = Robots[RobotID]->SystemVars[_ypos];
    R1_Y2 = Robots[RobotID]->SystemVars[_ypos];
    R1_X1 = MyDouble_SUB(R1_X1, FloatToMyDouble((ROBOT_SIZE / 2)));
    R1_X2 = MyDouble_ADD(R1_X2, FloatToMyDouble((ROBOT_SIZE / 2)));
    R1_Y1 = MyDouble_SUB(R1_Y1, FloatToMyDouble((ROBOT_SIZE / 2)));
    R1_Y2 = MyDouble_ADD(R1_Y2, FloatToMyDouble((ROBOT_SIZE / 2)));

    UPDATE_EVENT_STATE(&TempState, 1);

    if(Robots[RobotID]->Alive && Robots[Temp]->Alive)
    {
        //test this robot against the other robot
        MyDouble R2_X1, R2_X2, R2_Y1, R2_Y2;
        R2_X1 = Robots[Temp]->SystemVars[_xpos];
        R2_X2 = Robots[Temp]->SystemVars[_xpos];
        R2_Y1 = Robots[Temp]->SystemVars[_ypos];
        R2_Y2 = Robots[Temp]->SystemVars[_ypos];
        R1_X1 = MyDouble_SUB(R1_X1, FloatToMyDouble((ROBOT_SIZE / 2)));
        R1_X2 = MyDouble_ADD(R1_X2, FloatToMyDouble((ROBOT_SIZE / 2)));
        R1_Y1 = MyDouble_SUB(R1_Y1, FloatToMyDouble((ROBOT_SIZE / 2)));
        R1_Y2 = MyDouble_ADD(R1_Y2, FloatToMyDouble((ROBOT_SIZE / 2)));

        //look for any of the edges overlapping
        if((MyDouble_GREATERTHAN_EQUAL(R1_X1, R2_X1) && MyDouble_LESSTHAN(R1_X1, R2_X2)) || (MyDouble_GREATERTHAN(R1_X2, R2_X1) && MyDouble_LESSTHAN_EQUAL(R1_X2, R2_X2)))
        {
            //X's overlap, check Y's
            if((MyDouble_GREATERTHAN_EQUAL(R1_Y1, R2_Y1) && MyDouble_LESSTHAN(R1_Y1, R2_Y2)) || (MyDouble_GREATERTHAN(R1_Y2, R2_Y1) && MyDouble_LESSTHAN_EQUAL(R1_Y2, R2_Y2)))
            {
                //both collide so damage both
                Robots[RobotID]->SystemVars[_energy] = MyDouble_SUB(Robots[RobotID]->SystemVars[_energy], ImmToMyDouble(1));
                if(MyDouble_LESSTHAN(Robots[RobotID]->SystemVars[_energy], MyDouble0) || MyDouble_EQUAL(Robots[RobotID]->SystemVars[_energy], MyDouble0))
                    Robots[RobotID]->Alive = 0;

                Robots[Temp]->SystemVars[_energy] = MyDouble_SUB(Robots[Temp]->SystemVars[_energy], ImmToMyDouble(1));
                if(MyDouble_LESSTHAN(Robots[Temp]->SystemVars[_energy], MyDouble0) || MyDouble_EQUAL(Robots[Temp]->SystemVars[_energy], MyDouble0))
                    Robots[Temp]->Alive = 0;

		//if the robot is handling collisions then add this collision
                if(Robots[RobotID]->Events[EVENT_ROBOTCOLLISION] && Robots[RobotID]->Events[EVENT_ROBOTCOLLISION]->Function)
                {
                    //calculate the bearing
                    double x, y;
                    x = MyDoubleToDouble(MyDouble_SUB(Robots[Temp]->SystemVars[_xpos], Robots[RobotID]->SystemVars[_xpos]));
                    y = MyDoubleToDouble(MyDouble_SUB(Robots[Temp]->SystemVars[_ypos], Robots[RobotID]->SystemVars[_ypos]));
                    x = (atan2(x, y) * RADIAN_TO_DEG);
                    if(x < 0.0)
                        x = 360.0 + x;

                    //adjust for angle of the body
                    x = x - MyDoubleToDouble(Robots[RobotID]->SystemVars[_bodyaim]);

                    //now convert back to -180/+180
                    if(x > 180.0)
                        x -= 360.0;
                    else if(x < -180.0)
                        x += 360.0;

                    //setup the heading and bearing
                    Robots[RobotID]->Events[EVENT_ROBOTCOLLISION]->Triggers[Robots[RobotID]->Events[EVENT_ROBOTCOLLISION]->TriggerCount].Heading = Robots[Temp]->SystemVars[_bodyaim];
                    Robots[RobotID]->Events[EVENT_ROBOTCOLLISION]->Triggers[Robots[RobotID]->Events[EVENT_ROBOTCOLLISION]->TriggerCount].Bearing = DoubleToMyDouble(x);
                    Robots[RobotID]->Events[EVENT_ROBOTCOLLISION]->TriggerCount++;
                }

                if(Robots[Temp]->Events[EVENT_ROBOTCOLLISION] && Robots[Temp]->Events[EVENT_ROBOTCOLLISION]->Function)
                {
                    //calculate the bearing
                    double x, y;
                    x = MyDoubleToDouble(MyDouble_SUB(Robots[RobotID]->SystemVars[_xpos], Robots[Temp]->SystemVars[_xpos]));
                    y = MyDoubleToDouble(MyDouble_SUB(Robots[RobotID]->SystemVars[_ypos], Robots[Temp]->SystemVars[_ypos]));
                    x = (atan2(x, y) * RADIAN_TO_DEG);
                    if(x < 0.0)
                        x = 360.0 + x;

                    //adjust for angle of the body
                    x = x - MyDoubleToDouble(Robots[Temp]->SystemVars[_bodyaim]);

                    //now convert back to -180/+180
                    if(x > 180.0)
                        x -= 360.0;
                    else if(x < -180.0)
                        x += 360.0;

                    //setup the heading and bearing
                    Robots[Temp]->Events[EVENT_ROBOTCOLLISION]->Triggers[Robots[Temp]->Events[EVENT_ROBOTCOLLISION]->TriggerCount].Heading = Robots[RobotID]->SystemVars[_bodyaim];
                    Robots[Temp]->Events[EVENT_ROBOTCOLLISION]->Triggers[Robots[Temp]->Events[EVENT_ROBOTCOLLISION]->TriggerCount].Bearing = DoubleToMyDouble(x);
                    Robots[Temp]->Events[EVENT_ROBOTCOLLISION]->TriggerCount++;
                }

		//figure out the distance we need between the robots
		MyDouble DiffDistance = MyDouble_SUB(FloatToMyDouble(ROBOT_SIZE + 0.1), CalcDistance(Robots[RobotID]->SystemVars[_xpos], Robots[RobotID]->SystemVars[_ypos], Robots[Temp]->SystemVars[_xpos], Robots[Temp]->SystemVars[_ypos]));

		//give the robots 65% of the distance to allow a bit of a bounce off of each other
		DiffDistance = MyDouble_MUL(DiffDistance, FloatToMyDouble(0.65));

		//if this robot was not moving then use the heading of the other robot
		//otherwise use it's own heading to see how far to move
		MyDouble Diff, Heading;
		if(MyDouble_EQUAL(Robots[RobotID]->SystemVars[_moving], MyDouble0))
			Heading = Robots[Temp]->SystemVars[_bodyaim];
		else
		{
			Heading = MyDouble_ADD(Robots[RobotID]->SystemVars[_bodyaim], ImmToMyDouble(180));
			if(MyDouble_GREATERTHAN(Heading, ImmToMyDouble(360)))
				Heading = MyDouble_SUB(Heading, ImmToMyDouble(360));
		}

		Diff = MyDouble_MUL(DoubleToMyDouble(sin(MyDoubleToDouble(Heading) * DEG_TO_RADIAN)), DiffDistance);

		//adjust the first robot's X
		Robots[RobotID]->SystemVars[_xpos] = MyDouble_ADD(Robots[RobotID]->SystemVars[_xpos], Diff);
                R1_X1 = MyDouble_ADD(R1_X1, Diff);
                R1_X2 = MyDouble_ADD(R1_X2, Diff);

		//calculate Y
		Diff = MyDouble_MUL(DoubleToMyDouble(cos(MyDoubleToDouble(Heading) * DEG_TO_RADIAN)), DiffDistance);

		//adjust the first robot's X
		Robots[RobotID]->SystemVars[_ypos] = MyDouble_ADD(Robots[RobotID]->SystemVars[_ypos], Diff);
                R1_Y1 = MyDouble_ADD(R1_Y1, Diff);
                R1_Y2 = MyDouble_ADD(R1_Y2, Diff);

		//now the 2nd robot
		if(MyDouble_EQUAL(Robots[Temp]->SystemVars[_moving], MyDouble0))
			Heading = Robots[RobotID]->SystemVars[_bodyaim];
		else
		{
			Heading = MyDouble_ADD(Robots[RobotID]->SystemVars[_bodyaim], ImmToMyDouble(180));
			if(MyDouble_GREATERTHAN(Heading, ImmToMyDouble(360)))
				Heading = MyDouble_SUB(Heading, ImmToMyDouble(360));
		}

		Diff = MyDouble_MUL(DoubleToMyDouble(sin(MyDoubleToDouble(Heading) * DEG_TO_RADIAN)), DiffDistance);

		//adjust the first robot's X
		Robots[Temp]->SystemVars[_xpos] = MyDouble_ADD(Robots[Temp]->SystemVars[_xpos], Diff);

		//calculate Y
		Diff = MyDouble_MUL(DoubleToMyDouble(cos(MyDoubleToDouble(Heading) * DEG_TO_RADIAN)), DiffDistance);

		//adjust the first robot's X
		Robots[Temp]->SystemVars[_ypos] = MyDouble_ADD(Robots[Temp]->SystemVars[_ypos], Diff);


                //report both robot locations and the collision
                my_printf("RC %d %f %f %f\n", RobotID, Robots[RobotID]->SystemVars[_energy], Robots[RobotID]->SystemVars[_xpos], Robots[RobotID]->SystemVars[_ypos]);
                my_printf("RC %d %f %f %f\n", Temp, Robots[Temp]->SystemVars[_energy], Robots[Temp]->SystemVars[_xpos], Robots[Temp]->SystemVars[_ypos]);

                //we keep checking other robots as we need to make sure we don't have multiple robots hit at the same time as they all need damage
            }
        }
    }

    //next robot test
    Temp++;
    if(Temp < RobotCount)
        SWAP_TO_STATE(&TempState);
    else
    {
        //hit end of the robot list, advance
        RobotID++;
        Temp = RobotID + 1;
        if(Temp < RobotCount)
        {
            //setup for the current robot
            R1_X1 = Robots[RobotID]->SystemVars[_xpos];
            R1_X2 = Robots[RobotID]->SystemVars[_xpos];
            R1_Y1 = Robots[RobotID]->SystemVars[_ypos];
            R1_Y2 = Robots[RobotID]->SystemVars[_ypos];
            R1_X1 = MyDouble_SUB(R1_X1, FloatToMyDouble(ROBOT_SIZE / 2));
            R1_X2 = MyDouble_ADD(R1_X2, FloatToMyDouble(ROBOT_SIZE / 2));
            R1_Y1 = MyDouble_SUB(R1_Y1, FloatToMyDouble(ROBOT_SIZE / 2));
            R1_Y2 = MyDouble_ADD(R1_Y2, FloatToMyDouble(ROBOT_SIZE / 2));

            //start testing again
            SWAP_TO_STATE(&TempState);
        }
    }

    //all collisions should be done, cycle through all robots looking for those that are actively scanning and check for anything that needs to be reported
    RobotID = 0;
    Temp = 1;
    volatile double RadarMin, RadarMax;
    RadarMin = (MyDoubleToDouble(Robots[RobotID]->SystemVars[_radaraim]) - 8.0);
    if(RadarMin < 0.0)
        RadarMin += 360.0;
    RadarMax = (MyDoubleToDouble(Robots[RobotID]->SystemVars[_radaraim]) + 8.0);
    if(RadarMax >= 360.0)
        RadarMax -= 360.0;

    UPDATE_EVENT_STATE(&TempState, 1);

    if(Robots[Temp]->Alive && Robots[RobotID]->Alive && Robots[RobotID]->ScanActive)
    {
        //we need to look for a 16 degree area, the maximum rotation of a radar is 15 degrees
        double X = MyDoubleToDouble(MyDouble_SUB(Robots[Temp]->SystemVars[_xpos], Robots[RobotID]->SystemVars[_xpos]));
        double Y = MyDoubleToDouble(MyDouble_SUB(Robots[Temp]->SystemVars[_ypos], Robots[RobotID]->SystemVars[_ypos]));
        double Angle = atan2(X, Y) * RADIAN_TO_DEG;

        //we need 0 to 360
        if(Angle < 0)
            Angle = 360.0 + Angle;

	//detected robot within the scan area
        //keep in mind that we need to test across the 0 boundary
        if(((RadarMin < RadarMax) && ((Angle >= RadarMin) && (Angle <= RadarMax))) ||
           ((RadarMin > RadarMax) && ((Angle >= RadarMin) || (Angle <= RadarMax))) )
	{
            //robot was detected, adjust for the bearing based on the direction the radar is facing
            Angle = Angle - MyDoubleToDouble(Robots[RobotID]->SystemVars[_radaraim]);

            //alert this robot to the detection if need be
            if(Robots[RobotID]->Events[EVENT_ROBOTDETECT] && Robots[RobotID]->Events[EVENT_ROBOTDETECT]->Function)
            {
                //setup the heading and bearing
                Robots[RobotID]->Events[EVENT_ROBOTDETECT]->Triggers[Robots[RobotID]->Events[EVENT_ROBOTDETECT]->TriggerCount].Heading = Robots[Temp]->SystemVars[_bodyaim];
                Robots[RobotID]->Events[EVENT_ROBOTDETECT]->Triggers[Robots[RobotID]->Events[EVENT_ROBOTDETECT]->TriggerCount].Bearing = DoubleToMyDouble(Angle);
                Robots[RobotID]->Events[EVENT_ROBOTDETECT]->Triggers[Robots[RobotID]->Events[EVENT_ROBOTDETECT]->TriggerCount].Distance = CalcDistance(Robots[RobotID]->SystemVars[_xpos], Robots[RobotID]->SystemVars[_ypos], Robots[Temp]->SystemVars[_xpos], Robots[Temp]->SystemVars[_ypos]);
                Robots[RobotID]->Events[EVENT_ROBOTDETECT]->Triggers[Robots[RobotID]->Events[EVENT_ROBOTDETECT]->TriggerCount].Energy = Robots[Temp]->SystemVars[_energy];
                Robots[RobotID]->Events[EVENT_ROBOTDETECT]->TriggerCount++;
            }

            //alert the other robot to the ping
            if(Robots[Temp]->Events[EVENT_PING] && Robots[Temp]->Events[EVENT_PING]->Function)
            {
                double x, y;
                x = MyDoubleToDouble(MyDouble_SUB(Robots[RobotID]->SystemVars[_xpos], Robots[Temp]->SystemVars[_xpos]));
                y = MyDoubleToDouble(MyDouble_SUB(Robots[RobotID]->SystemVars[_ypos], Robots[Temp]->SystemVars[_ypos]));
                x = (atan2(x, y) * RADIAN_TO_DEG);
                if(x < 0.0)
                    x = 360.0 + x;

                //adjust for angle of the body
                x = x - MyDoubleToDouble(Robots[Temp]->SystemVars[_bodyaim]);

                //now convert back to -180/+180
                if(x > 180.0)
                    x -= 360.0;
                else if(x < -180.0)
                    x += 360.0;

                //setup the heading and bearing
                Robots[Temp]->Events[EVENT_PING]->Triggers[Robots[Temp]->Events[EVENT_PING]->TriggerCount].Heading = Robots[RobotID]->SystemVars[_bodyaim];
                Robots[Temp]->Events[EVENT_PING]->Triggers[Robots[Temp]->Events[EVENT_PING]->TriggerCount].Bearing = DoubleToMyDouble(x);
                Robots[Temp]->Events[EVENT_PING]->TriggerCount++;   
            }
	}
    }

    //tested this robot, advance to the next one
    Temp++;
    if(Temp < RobotCount)
        SWAP_TO_STATE(&TempState);
    else
    {
        //hit end of the robot list, advance
        Robots[RobotID]->ScanActive = 0;
        RobotID++;
        Temp = (RobotID + 1) % RobotCount;

        //start testing again if we can
        if(RobotID < RobotCount)
        {
            RadarMin = (MyDoubleToDouble(Robots[RobotID]->SystemVars[_radaraim]) - 8.0);
            if(RadarMin < 0)
                RadarMin += 360.0;
            RadarMax = (MyDoubleToDouble(Robots[RobotID]->SystemVars[_radaraim]) + 8.0);
            if(RadarMax >= 360.0)
                RadarMax -= 360.0;

            SWAP_TO_STATE(&TempState);
        }
    }

    //all done
    SWAP_TO_STATE(&GlobalState);
}

void CreateRobotStates(RobotStruct *RobotData)
{
    //resposible for cycling through all of the states for a robot
    volatile int i;
    volatile StateStruct TempState;
    
    //copy the global state to the temp state
    memcpy(&TempState, &GlobalState, sizeof(StateStruct));

    //allocate all states    
    for(i = 0; i < MAX_EVENTS; i++)
    {
        RobotData->Events[i] = (StateStruct *)malloc(sizeof(StateStruct));
        memset(RobotData->Events[i], 0, sizeof(StateStruct));
    }

    //now setup our counter as HandleState will keep returning to the global state that is saved
    i = -1;

    //save off to the global state and swap to it
    SAVE_EVENT_STATE(&GlobalState, 1);
    
    //cycle through and create each state
    i += 1;
    if(i < MAX_EVENTS)
        HandleState(RobotData, i);

    //done creating states, setup the initial state function
    for(i = 0; i < RobotData->FunctionCount; i++)
    {
        if(my_strcmp(RobotData->Functions[i]->Name,"init") == 0)
            RobotData->Events[EVENT_INIT]->Function = RobotData->Functions[i];
        else if(my_strcmp(RobotData->Functions[i]->Name,"dead") == 0)
            RobotData->Events[EVENT_DEAD]->Function = RobotData->Functions[i];

        //if init and dead are assigned then stop looking
        if(RobotData->Events[EVENT_DEAD]->Function && RobotData->Events[EVENT_INIT]->Function)
            break;
    }
    
    //each of the states for the robot is created, mark the robot as alive
    RobotData->Alive = 1;
    
    //swap back
    memcpy(&GlobalState, &TempState, sizeof(StateStruct));
    SWAP_TO_STATE(&GlobalState);
}

void RunRobots(int RobotCount, RobotStruct **Robots)
{
    volatile int CurRobotEntry;
    volatile int CurEventLevel;
    volatile int i;
    volatile int Matched;
    volatile int AliveCount;
    volatile int RBCount;

    RBCount = RobotCount;
    CurRobotEntry = 0;
    CurEventLevel = -1;
    Matched = 0;
    AliveCount = 0;

    //this function won't ever exit
    SAVE_EVENT_STATE(&GlobalState, 1);

    //my_printf("global state: Robot %d, Matched %d, Event Level %d\n", CurRobotEntry, Matched, CurEventLevel);

    //if no match then we need to advance the level and possibly the robot
    if(Matched == 0)
    {
        CurEventLevel = (CurEventLevel + 1) % MAX_EVENTS;
        if(CurEventLevel >= (MAX_EVENTS - 2))
        {
            CurRobotEntry = (CurRobotEntry + 1) % RBCount;
            if(CurRobotEntry == 0)
            {
                //if 1 robot is alive then go find it so we can print the winner. We look for 2 counts as our sub system counts as alive
                if(AliveCount == 2)
                {
                    SAVE_EVENT_STATE(&GlobalState, 1);
                    if(Robots[CurRobotEntry]->Alive == 0)
                    {
                        CurRobotEntry++;
                        SWAP_TO_STATE(&GlobalState);
                    }

                    //print the winner and exit
                    my_printf("RW %d Robot %s wins\n", CurRobotEntry, Robots[CurRobotEntry]->Name);
                    exit(0);
                }
                AliveCount = 0;
            }
            CurEventLevel = 0;
        }
       
        //if first time then do some extra checks
        if(CurEventLevel == 0)
        {
            //if the robot is not initialized then init it
            if(!Robots[CurRobotEntry]->Initialized)
            {
                //reset the event level so the core can trigger
                CurEventLevel = -1;
                
                //init the robot
                Robots[CurRobotEntry]->Initialized = 1;
                if(Robots[CurRobotEntry]->Events[EVENT_INIT] && Robots[CurRobotEntry]->Events[EVENT_INIT]->Function)
                    SWAP_TO_STATE(Robots[CurRobotEntry]->Events[EVENT_INIT]);
                else
                    Robots[CurRobotEntry]->Alive = 0;
            }
            
            //if this robot is dead then ignore it
            if(!Robots[CurRobotEntry]->Alive)
            {
                //make sure we advance robots when we come back through
                CurEventLevel = (MAX_EVENTS - 2);
                
                //if we have a dead event then process it
                if(Robots[CurRobotEntry]->Initialized == 1)
                {
                    //report the death
                    my_printf("RD %d\n", CurRobotEntry);

                    //robot died, increment the init'd variable as a flag so we only do this once
                    Robots[CurRobotEntry]->Initialized++;
                    if(Robots[CurRobotEntry]->Events[EVENT_DEAD] && Robots[CurRobotEntry]->Events[EVENT_DEAD]->Function)
                    {
                        Robots[CurRobotEntry]->Events[EVENT_DEAD]->TriggerCount = 1;
                        SWAP_TO_STATE(Robots[CurRobotEntry]->Events[EVENT_DEAD]);
                    }
                }

                SWAP_TO_STATE(&GlobalState);
            }
            else
            {
                //make sure we report robot energy
                if((CurRobotEntry != (RBCount - 1)) && !MyDouble_EQUAL(Robots[CurRobotEntry]->OldEnergy, Robots[CurRobotEntry]->SystemVars[_energy]))
                {
                    my_printf("RE %d %f\n", CurRobotEntry, Robots[CurRobotEntry]->SystemVars[_energy]);
                    Robots[CurRobotEntry]->OldEnergy = Robots[CurRobotEntry]->SystemVars[_energy];
                }
                AliveCount++;
            }

            //if robot is stalled then don't do anything
            if(Robots[CurRobotEntry]->Stalled)
            {
                Robots[CurRobotEntry]->Stalled--;
                if(Robots[CurRobotEntry]->Stalled < 0)
                    Robots[CurRobotEntry]->Stalled = 0;
                
                //make sure we advance robots when we come back through
                CurEventLevel = MAX_EVENTS - 3;

                SWAP_TO_STATE(&GlobalState);
            }

            //if the core event exists then fire it
            if(Robots[CurRobotEntry]->Events[EVENT_CORE] && Robots[CurRobotEntry]->Events[EVENT_CORE]->Function)
            {
                CurEventLevel = 0;
                Robots[CurRobotEntry]->Events[EVENT_CORE]->TriggerCount = 1;
                SWAP_TO_STATE(Robots[CurRobotEntry]->Events[EVENT_CORE]);
            }
            
            //if the robot is moving then make sure the autoscan will fire otherwise turn it off
            if(Robots[CurRobotEntry]->SystemVars[_moving].SubNumber)
                Robots[CurRobotEntry]->Events[EVENT_AUTOSCAN]->TriggerCount = 1;
            else
                Robots[CurRobotEntry]->Events[EVENT_AUTOSCAN]->TriggerCount = 0;
        }
        
        Matched = EVENT_CORE + 1;
    }
    
    //look for any events that match the current level
    for(i = Matched; i < MAX_EVENTS; i++)
    {
        //if the event is assigned and matches this level and has a function and has trigger events then call it
        if(Robots[CurRobotEntry]->Events[i] && (Robots[CurRobotEntry]->Events[i]->EventLevel == CurEventLevel) && Robots[CurRobotEntry]->Events[i]->Function && (Robots[CurRobotEntry]->Events[i]->TriggerCount || (Robots[CurRobotEntry]->Events[i]->CurState != IDLE_STATE)))
        {
            Matched = i + 1;
            SWAP_TO_STATE(Robots[CurRobotEntry]->Events[i]);
        }
    }
    
    //no more matches, try again
    Matched = 0;
    SWAP_TO_STATE(&GlobalState);
}

void FREE_STATE(StateStruct *State)
{
    if(State->Stack)
    {
        free(State->Stack);
        State->Stack = 0;
    }
}

void AlarmTrigger(int signum)
{
    //all done
    write(1, "\nAlarm triggered\n", 17);
    fflush(stdout);
    exit(0);
}

int ReadLine(int fd, char *Buffer)
{
    //BUG: I fail to limit the input length and only look for newline
    int Count = 0;
    int TotalCount = 0;
    while(1)
    {
        Count = read(fd, Buffer, 1);
        if(Count > 0)
        {
            TotalCount += Count;
            if(*Buffer == '\n')
                return TotalCount;
            Buffer += Count;
        }
    };

    return TotalCount;
}

int ReadAll(int fd, char *Buffer, int Size)
{
    //read the number of bytes specified
    int Count = 0;
    int TotalCount = 0;
    while(Size)
    {
        Count = read(fd, Buffer, Size);
        if(Count > 0)
        {
            Size -= Count;
            Buffer += Count;
            TotalCount += Count;
        }
    };

    return TotalCount;
}

int main(int argc, char **argv)
{
    char Buffer[256];
    char *RobotBuf;
    unsigned int size;
    volatile RobotStruct **Robot;
    volatile int RobotsCreated = 0;
    volatile unsigned int CurRobotID = 0;
    volatile unsigned int RobotCount;
    volatile int fd = 0;

    //give 20 seconds to have 1 remaining robot
    signal(SIGALRM, AlarmTrigger);
    alarm(20);

    //get the seed from the command line
    size = ReadLine(fd, Buffer);
    if(size < sizeof(Buffer))
        Buffer[size] = 0;
    Buffer[sizeof(Buffer) - 1] = 0;
    size = my_atoi(Buffer);

    srand(size);
    my_printf("Seed: %d\n", size);

    size = ReadLine(fd, Buffer);
    Buffer[size] = 0;
    RobotCount = my_atoi(Buffer);

    if(RobotCount > 10)
    {
        my_printf("Maximum of 10 robots allowed\n");
        exit(0);
    }

    my_printf("Processing %d robots\n", RobotCount);

    RobotCount += 2;
    Robot = malloc(sizeof(RobotStruct*) * RobotCount);

    CREATE_GLOBAL_STATE(&GlobalState);
    CurRobotID++;

    //if on the last robot then read from FD 4
    if(CurRobotID == (RobotCount - 1))
        fd = 1001;

    if(CurRobotID == RobotCount)
    {
        my_printf("Processed %d robots\n", RobotsCreated);
    }
    else if(CurRobotID < RobotCount)
    {
        //get how many bytes of data are for this robot
        Buffer[0] = 0;
        size = ReadLine(fd, Buffer);
        Buffer[size] = 0;
        size = atoi(Buffer);

        if(size > (64*1024))
        {
            my_printf("Robot too large\n");
            fflush(stdout);
            exit(0);
        }

        my_printf("Robot %d - %d bytes\n", CurRobotID, size);
        RobotBuf = malloc(size + 1);
        ReadAll(fd, RobotBuf, size);
        RobotBuf[size] = 0;
        
        Robot[RobotsCreated] = ParseRobot(RobotBuf);
        if(Robot[RobotsCreated]->Initialized != -1)
	{
	        RobotsCreated++;
        	CreateRobotStates(Robot[RobotsCreated-1]);
	}

	my_printf("Skipping robot %d\n", CurRobotID);
        SWAP_TO_STATE(&GlobalState);
    }

    //create an entry that handles all main events, missiles, etc
    if(CurRobotID == RobotCount)
    {
        //create our system robot then go do the init for the system handler
        Robot[RobotsCreated] = malloc(sizeof(RobotStruct));
        memset(Robot[RobotsCreated], 0, sizeof(RobotStruct));
        
        RobotsCreated++;
        HandleSystemEvents(RobotsCreated - 1, Robot, Robot[RobotsCreated - 1]);
    }

    for(fd = 0; fd < (RobotsCreated - 1); fd++)
    {
        Robot[fd]->SystemVars[_xpos] = ImmToMyDouble(rand() % 16300);
        Robot[fd]->SystemVars[_ypos] = ImmToMyDouble(rand() % 16300);
        Robot[fd]->OldX = Robot[fd]->SystemVars[_xpos];
        Robot[fd]->OldY = Robot[fd]->SystemVars[_ypos];

        //make sure the robot has a name
        if(!Robot[fd]->Name[0])
            my_snprintf(Robot[fd]->Name, sizeof(Robot[fd]->Name), "@Robot%d", fd);

        my_printf("Robot %d start: %s %f %f\n", fd, Robot[fd]->Name, Robot[fd]->SystemVars[_xpos], Robot[fd]->SystemVars[_ypos]);
    }

    RunRobots(RobotsCreated, Robot);
    return 0;
}
