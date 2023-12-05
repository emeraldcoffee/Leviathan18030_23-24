package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

//Use this file to set up values that will be used by the robot like servo positions
public class RobotConstants {

    //Whole # variables
    public static final int
        slideBottom         = (int)(8192 * 0),
        slideLow            = 8192 * 2,
        slideMiddle         = 8192 * 4,
        slideTop            = 8192 * 6;



    //Double variables
    public static final double

    //Left + right servo limits
        leftOut = .81,
        leftIn = .62,
        rightOut = .31,
        rightIn = .6,


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
        dropTime        = .09,
        resetTime       = .22,
        dropOpen        = .73,
        dropClosed      = .59,

        transferSpeed = 1,
        intakeSpeed     = .6,

        climbReleaseDelay = 100,
        intakeReverseDelay = 300,
        intakeReverseTime = 200,
        climbSpeed   = 1,
        climbReleaseSpeed = .6,
    //Pixel tracking variables
        intakeValue          = 3,
        outtakeValue     = 50,
        outtakeCountDelay = 200;

    public static final PIDCoefficients slidePIDVals = new PIDCoefficients(5.0 / 8192, .03 / 8192, .07 / 8192);

}
