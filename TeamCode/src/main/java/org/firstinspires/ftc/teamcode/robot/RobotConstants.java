package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

//Use this file to set up values that will be used by the robot like servo positions
public class RobotConstants {

    //Whole # variables
    public static final int
        slideBottom         = (int)(8192 * 0.01),
        slideAuto = (int)(8192 * 2.1),
        slideLow            = (int)(8192 * 3),
        slideMiddle         = (int) (8192 * 4.2),
        slideTop            = (int)(8192 * 6);



    //Double variables
    public static final double

    //Left + right servo limits
        leftOut = .86,
        leftIn = .5,
        rightOut = .31,
        rightIn = .6,

        droneRelease = .65,


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
        resetTime       = .27,
        dropOpen        = .71,
        dropClosed      = .63,

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
