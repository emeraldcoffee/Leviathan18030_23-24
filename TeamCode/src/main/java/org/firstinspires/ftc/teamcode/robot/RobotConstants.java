package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

//Use this file to set up values that will be used by the robot like servo positions
public class RobotConstants {

    //Whole # variables
    public static final int
        Finalthing          = 5,
        Otherfinalthing     = 3,
        slideBottom         = 0,
        slideGround         = (int)(8192*1.3),
        slideLow            = 8192 * 3,
        slideMiddle         = (int)(8192 * 4.5),
        slideTop            = (int)(8192 * 6.1),
        slideMinHeight      = 0,
        slideMaxHeight      = (int)(8192 * 6.1),
        slideGround         = (int)(8192 * 1.3);


    //Double variables
    public static final double
        //Drivetrain constants
        speedMultiplier = 0.6,
        maxVelo         = 20000,
        maxAccel        = 300,
        driveSpeed      = 1,
        strafeSpeed     = 1,
        turnSpeed       = .6,

        dropTime        = .18,
        resetTime       = .27,
        dropOpen        = .7,
        dropClosed      = .5,
        transferSpeed   = 1.0,
        intakeSpeed     = 0.7,
        climbSpeed      = 1.0;

    public static final PIDCoefficients slidePIDVals = new PIDCoefficients(0.8 / 8192, .01 / 8192, .001 / 8192);


    public static final PIDCoefficients
            slidePIDVals = new PIDCoefficients(0.8 / 8192, .01 / 8192, .001 / 8192);

}
