package org.firstinspires.ftc.teamcode.robot;

//Use this file to set up values that will be used by the robot like servo positions
public class RobotConstants {

    //Whole # variables
    public static final int
        slideBottom         = (int)(8192 * .1),
        slideLow            = 8192 * 2,
        slideMiddle         = 8192 * 4,
        slideTop            = 8192 * 6;



    //Double variables
    public static final double

    //Drivetrain variables
        speedMultiplier = .6,
        maxVelo         = 1,
        maxAccel        = 10,
        driveSpeed      = 1,
        strafeSpeed     = 1,
        turnSpeed       = .6,

    //Backdrop align variables
        backDropAlignX = 42,
        backDropX = 50,
        backDropLeftY = 35.2,
        backDropRightY = -35.2,

    //Outtake variables
        dropTime        = .12,
        resetTime       = .27,
        dropOpen        = .7,
        dropClosed      = .5,

        transferSpeed = 1,
        intakeSpeed     = .6,
        intakeReverseDelay = 300,
        intakeReverseTime = 200,
        climbSpeed   = .6,
    //Pixel tracking variables
        intakeValue          = 3,
        outtakeValue     = 50,
        outtakeCountDelay = 200;


}
