package org.firstinspires.ftc.teamcode.robot;

import static java.lang.Math.abs;
import static java.lang.Math.max;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class RobotMethods {

    double accelTime;
    double halfDist;
    double accelDist;
    double decelDist;
    double constDist;
    double constTime;
    double decelTime;

    //Takes input values and sets drivetrain to corresponding powers while scaling all powers under 1
    public static void setMecanumDrive(double forward, double strafe, double turn, SampleMecanumDrive driveTrain) {
        //Find value to make all motor powers less than 1
        double scalePower = max(abs(forward) + abs(strafe) + abs(turn), 1);

        //Creating string with all drive powers for mecanum drive
        Double[] driveSpeeds = {(forward-strafe-turn)/scalePower, (forward+strafe-turn)/scalePower,
                                (forward-strafe+turn/scalePower), (forward+strafe+turn)/scalePower};

        //Setting motors to there new powers
        driveTrain.setMotorPowers(driveSpeeds[0], driveSpeeds[1], driveSpeeds[2], driveSpeeds[3]);
    }

    //Takes input values and sets drivetrain to corresponding powers while scaling all powers under maxPower
    public static void setMecanumDrive(double forward, double strafe, double turn, double maxPower, SampleMecanumDrive driveTrain) {
        //Find value to make all motor powers less than 1
        double scalePower = max(abs(forward) + abs(strafe) + abs(turn), maxPower);

        //Creating string with all drive powers for mecanum drive
        Double[] driveSpeeds = {(forward-strafe-turn)/scalePower, (forward+strafe-turn)/scalePower,
                                (forward-strafe+turn/scalePower), (forward+strafe+turn)/scalePower};

        //Setting motors to there new powers
        driveTrain.setMotorPowers(driveSpeeds[0], driveSpeeds[1], driveSpeeds[2], driveSpeeds[3]);
    }



    //Takes input values and sets drivetrain to corresponding field-centric powers while scaling all powers under 1
    public static void setMecanumDriveFieldCentric(double forward, double strafe, double turn, double heading, SampleMecanumDrive driveTrain) {

        //rotating Joystick values to account for robot heading
        double rotatedForward = forward*Math.cos(-heading)-strafe*Math.sin(-heading),
                rotatedStrafe = forward*Math.cos(-heading)+strafe*Math.sin(-heading);

        //Find value to make all motor powers less than 1
        double scalePower = max(abs(rotatedForward) + abs(rotatedStrafe) + abs(turn), 1);

        //Creating string with all drive powers for mecanum drive
        Double[] driveSpeeds = {rotatedForward+rotatedStrafe+turn/scalePower, rotatedForward-rotatedStrafe+turn/scalePower, rotatedForward-rotatedStrafe-turn/scalePower, rotatedForward+rotatedStrafe-turn/scalePower};

        //Setting motors to their new powers
        driveTrain.setMotorPowers(driveSpeeds[0], driveSpeeds[1], driveSpeeds[2], driveSpeeds[3]);
    }

    //Takes input values and sets drivetrain to corresponding field-centric powers while scaling all powers under maxPower
    public static void setMecanumDriveFieldCentric(double forward, double strafe, double turn, double maxPower, double heading, SampleMecanumDrive driveTrain) {

        //rotating Joystick values to account for robot heading
        double rotatedForward = forward*Math.cos(-heading)-strafe*Math.sin(-heading),
               rotatedStrafe = forward*Math.cos(-heading)+strafe*Math.sin(-heading);

        //Find value to make all motor powers less than maxPower
        double scalePower = max(abs(rotatedForward) + abs(rotatedStrafe) + abs(turn), maxPower);

        //Creating string with all drive powers for mecanum drive
        Double[] driveSpeeds = {rotatedForward+rotatedStrafe+turn/scalePower, rotatedForward-rotatedStrafe+turn/scalePower, rotatedForward-rotatedStrafe-turn/scalePower, rotatedForward+rotatedStrafe-turn/scalePower};

        //Setting motors to their new powers
        driveTrain.setMotorPowers(driveSpeeds[0], driveSpeeds[1], driveSpeeds[2], driveSpeeds[3]);
    }

    //Adds telemetry data for robot position
    public static String updateRobotPosition(Pose2d pose) {
        return "x: " + pose.getX() + " y: " + pose.getY() + " heading: " + pose.getHeading();
    }

    public static void outtakePlace (hardwareMap hwMap) {
        ElapsedTime dropTime = new ElapsedTime();
        hwMap.dropServo.setPosition(RobotConstants.dropOpen);
        dropTime.reset();
        if (dropTime.seconds() > RobotConstants.dropTime) {
            hwMap.dropServo.setPosition(RobotConstants.dropClosed);
        }
    }

    public void setTargetPos(double maxAccel, double maxVelo, int finalPos) {
        accelTime = maxVelo / maxAccel;
        halfDist = finalPos / 2;
        accelDist = .5 * maxAccel * Math.pow(accelTime, 2);

        if (accelDist > halfDist) {
            accelDist = Math.sqrt(halfDist / (.5 * maxAccel));
        }
        accelDist = .5 * maxAccel * Math.pow(accelTime, 2);

        maxVelo =  maxAccel * accelDist; // changes max velo based on max accel and dist to accel

        decelDist = accelDist;
        constDist = finalPos - (accelDist * 2);
        constTime = constDist / maxVelo;
        decelTime = accelTime + constTime;
    }

    public double slidesUpdate(double maxAccel, double maxVelo, int finalPos) {
        ElapsedTime timer = new ElapsedTime();
        double constCurrTime;
        setTargetPos(maxAccel, maxVelo, finalPos);
        double totalTime = accelTime + constTime + decelTime;
        if (timer.seconds() > totalTime)
            return finalPos;
        if (timer.seconds() < accelTime)
            return Math.pow(.5 * maxAccel * timer.seconds(), 2);
        else if (timer.seconds() < decelTime) {
            accelDist = Math.pow(.5 * maxAccel * accelTime, 2);
            constCurrTime = timer.seconds() - accelTime;
            return accelDist + maxVelo * constCurrTime;
        }
        else {
            accelDist = Math.pow(.5 * maxAccel * accelTime, 2);
            constDist = maxVelo * constTime;
            decelTime = timer.seconds() - decelTime;

            return Math.pow(accelDist + constDist + maxVelo * decelTime - .5 * maxAccel * decelTime, 2);
        }
    }
    /*public static double slidePID(hardwareMap hwMap, double kP, double kI, double kD, double distance) { // tuning and desired (in ticks)

        ElapsedTime timer = new ElapsedTime();

        double startPos = hwMap.liftEncoder.getCurrentPosition();
        double currPos = 0.0;
        double runTimeOne = 0.0;
        double runTimeTwo = 0.0;
        double error = 0.0;

        double p = 0.0;
        double i = 0.0;
        double d = 0.0;

        while (Math.abs(currPos - distance) > 10) { // 10 is the amount of error
            currPos = hwMap.liftEncoder.getCurrentPosition() - startPos;
            runTimeOne = timer.seconds();
            error = distance - currPos;

            p = error * kP;
            runTimeTwo = timer.seconds();
            i = error * (runTimeTwo - runTimeOne) * kI;
            d = error / (runTimeTwo - runTimeOne) * kD;
        }

        return p + i + d;
    }


    public static void slideExtend (hardwareMap hwMap, double distance) { // needs to go from inches to ticks

        hwMap.liftMotor.setPower(slidePID(hwMap, .2, .2, .2, distance));
        outtakePlace(hwMap);

        slideRetract(hwMap, distance);
    }

    public static void slideRetract (hardwareMap hwMap, double distance) {
        hwMap.liftMotor.setPower(-slidePID(hwMap, .2, .2, .2, distance));
    }*/
}
