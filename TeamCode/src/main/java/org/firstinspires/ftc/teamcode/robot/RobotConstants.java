package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

//Use this file to set up values that will be used by the robot like servo positions
public class RobotConstants {

    //Whole # variables
    public static final int
        slideBottom         = (int)(8192 * 0.01),
        slideAuto           = (int)(8192 * 2.1),
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
        doubleDropTime = .27,
        resetTime       = .16,
        dropOpen        = .71,//71
        dropPartial = .6275,//
        dropClosed      = .545,//545


        transferSpeed = 1,
        intakeSpeed     = 1,
    //Intake auto stack positions
        intakeAdjustDelayTime = .2,
        stackDrawbridgeDown = .28,
        stackDrawbridgeUp = .11,
        drawbridgeRightOffset = .02,
        drawbridgeLeftOffset = 0,
        stackMax = .38,
        stack5 = .234,
        stack4 = .202,
        stack3 = .181,
        stack2 = .153,
        stack1 = .115,
        stackLeftOffset = .023,

        climbReleaseDelay = 100,


    //Auto variables
        intakeInitalizeDelay = .6,
        autoDropTime = 20,
        intakeMaxSpinTime = 800,
        autoTransferTime = 4;

    public static final PIDCoefficients slidePIDVals = new PIDCoefficients(1.575 / 8192, .005 / 8192, .038 / 8192);

}
