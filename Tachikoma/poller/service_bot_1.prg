Core
{
    if( _energy <= 1 )
        print( "DIE" )
        suicide( )
    endif

    if( Firing == 0 )
        print( "Rotating radar" )
        radarright( 15 )
        scan( )
        endturn( )
    else
        waitfor( DetectDone == 1)
        Firing = 0
        DetectDone = 0
        scan( )
    endif
}

RobotCld
{
    print( "collision: bearing" _cldbearing "heading" _cldheading )
    stopmove( )
}

RobotDetect
{
    print( "Found robot" _dtcbearing "heading" _dtcheading "distance" _dtcdistance "energy" _dtcenergy )
    print( "Radar" _radaraim "gun" _gunaim )
    Firing = 1
    Distance = _dtcdistance

    #point radar at robot
    if( _dtcbearing < 0 )
        abs( _dtcbearing )
        radarleft( _result )
    else
        radarright( _dtcbearing )
    endif

    #point the gun at the robot
    GunFacing = _gunaim - _radaraim
    if( GunFacing < -180.0 )
        GunFacing = GunFacing + 360
    elseif( GunFacing > 180 )
        GunFacing = GunFacing - 360
    endif

    if( GunFacing < 0 )
        abs( GunFacing )
        gunright( _result )
    else
        gunleft( GunFacing )
    endif

    #if too far then move towards the robot
    if( Distance > 3000 )
        BodyTurn = _gunaim - _bodyaim

        if( BodyTurn < -180.0 )
            BodyTurn = BodyTurn + 360
        elseif( BodyTurn > 180.0 )
            BodyTurn = BodyTurn - 360
        endif

        if( BodyTurn >= 0 )
            bodyright( BodyTurn )
        else
            abs( BodyTurn )
            bodyleft( _result )
        endif
        MoveDist = 800
        ahead( MoveDist )
    else
        fire( 1 )
        waitfor( _gunheat == 0 )
    endif

    DetectDone = 1
}

Init
{
    regcore( "Core" )
    regcldrobot( "RobotCld", 1 )
    regdtcrobot( "RobotDetect", 2 )
}
