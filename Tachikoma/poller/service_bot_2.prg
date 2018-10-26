Core
{
    getrandom( )
    RotateDir = _result % 180
    if( RotateDir > 90 )
        bodyright( RotateDir - 90 )
    else
        bodyleft( RotateDir )
    endif

    getrandom( )
    Distance = _result % 200
    ahead( Distance )
}

RobotCld
{
    print( "collision: bearing" _cldbearing "heading" _cldheading )
    stopmove( )
}

Init
{
    regcore( "Core" )
    regcldrobot( "RobotCld", 1 )
}
