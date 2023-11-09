package org.firstinspires.ftc.teamcode.robot;

//Use this file to set up values that will be used by the robot like servo positions
public class RobotConstants {

    //Whole # variables
    public static final int
        Finalthing          = 5,
        Otherfinalthing     = 3,
        slideBottom         = 0,
        slideLow            = 8192 * 3,
        slideMiddle         = (int)(8192 * 4.5),
        slideTop            = (int)(8192 * 6.1),
        slideMinHeight      = 0,
        slideMaxHeight      = (int)(8192 * 6.1);


    //Double variables
    public static final double
        //Drivetrain constants
        speedMultiplier = 0.6,
        maxVelo         = 20000,
        maxAccel        = 300,
        driveSpeed      = 1,
        strafeSpeed     = 1,
        turnSpeed       = .6,

        dropTime        = .12,
        resetTime       = .27,
        dropOpen        = .7,
        dropClosed      = .5,
        transferSpeed   = 1.0,
        intakeSpeed     = 1,
        climbSpeed      = 1.0;



}
