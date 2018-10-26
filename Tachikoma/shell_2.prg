#############################################################################
#                                  Shell                                    #
#                                                                           #
# This robot does absolutely nothing. It is meant to be a starting point    #
# for creating your own robots. The sections below are used by many robots. #
# Add code and change names as needed to make your own robot!               #
#                                                                           #
#############################################################################


AutoScan
{
}

Core
{

}  

MissileHit
{
    print( "Hit by missle" )
}

FoundRobot
{
}

Dead
{
}

Pinged
{
    print( "Pinged from" _cldbearing "heading" _cldheading )
}

Init
{
    name( "robot2" )
    regcore( "Core" )
    regcldmissile( "MissileHit", 1 )
    regdtcrobot( "FoundRobot", 2 )
    regascan( "AutoScan", 3 )
    regping( "Pinged", 1 )
    bodyright( 14.123531 )
}
