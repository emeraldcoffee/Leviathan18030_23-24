package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.RobotConstants.maxAccel;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.maxVelo;
import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.round;

import android.annotation.SuppressLint;
import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Locale;

public class RobotMethods {

    int startPosStore;
    int targetPosStore;
    double max_velocity;
    double acceleration_dt;
    double acceleration_distance;
    double deceleration_dt;
    double cruise_dt;
    double cruise_distance;
    double deceleration_time;
    ElapsedTime elapsed_time = new ElapsedTime();

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
                rotatedStrafe = forward*Math.sin(-heading)+strafe*Math.cos(-heading);

        //Find value to make all motor powers less than 1
        double scalePower = max(abs(rotatedForward) + abs(rotatedStrafe) + abs(turn), 1);

        //Creating string with all drive powers for mecanum drive
        Double[] driveSpeeds = {(rotatedForward-rotatedStrafe-turn)/scalePower, (rotatedForward+rotatedStrafe-turn)/scalePower,
                                (rotatedForward-rotatedStrafe+turn/scalePower), (rotatedForward+rotatedStrafe+turn)/scalePower};

        //Setting motors to their new powers
        driveTrain.setMotorPowers(driveSpeeds[0], driveSpeeds[1], driveSpeeds[2], driveSpeeds[3]);
    }

    //Takes input values and sets drivetrain to corresponding field-centric powers while scaling all powers under maxPower
    public static void setMecanumDriveFieldCentric(double forward, double strafe, double turn, double maxPower, double heading, SampleMecanumDrive driveTrain) {

        //rotating Joystick values to account for robot heading
        double rotatedForward = forward*Math.cos(-heading)-strafe*Math.sin(-heading),
               rotatedStrafe = forward*Math.sin(-heading)+strafe*Math.cos(-heading);

        //Find value to make all motor powers less than maxPower
        double scalePower = max(abs(rotatedForward) + abs(rotatedStrafe) + abs(turn), maxPower);

        //Creating string with all drive powers for mecanum drive
        Double[] driveSpeeds = {(rotatedForward-rotatedStrafe-turn)/scalePower, (rotatedForward+rotatedStrafe-turn)/scalePower,
                                (rotatedForward-rotatedStrafe+turn/scalePower), (rotatedForward+rotatedStrafe+turn)/scalePower};

        //Setting motors to their new powers
        driveTrain.setMotorPowers(driveSpeeds[0], driveSpeeds[1], driveSpeeds[2], driveSpeeds[3]);
    }

    public static void goToPoint(double targetX, double targetY, double targetHeading, SampleMecanumDrive driveTrain) {
        double forward = targetX-driveTrain.getPoseEstimate().getX()/4;
        double strafe = targetY-driveTrain.getPoseEstimate().getY()/4;
        double turn = targetHeading-driveTrain.getPoseEstimate().getHeading()/10;

        setMecanumDriveFieldCentric(forward, strafe, turn, 1, driveTrain.getPoseEstimate().getHeading(), driveTrain);

    }

    public static void goToLineY(double targetX, double powerY, double targetHeading, SampleMecanumDrive driveTrain) {
        double forward = targetX-driveTrain.getPoseEstimate().getX()/4;
        double turnPower = targetHeading-driveTrain.getPoseEstimate().getHeading()/10;

        if (turnPower>Math.PI) {
            turnPower -= 2*Math.PI;
        } else if (turnPower < -Math.PI) {
            turnPower += 2*Math.PI;
        }

        setMecanumDriveFieldCentric(forward, powerY, Range.clip(turnPower*2/*-turnVelocity*.16*/, -4, 4), 1, driveTrain.getPoseEstimate().getHeading(), driveTrain);

    }

    //Adds telemetry data for robot position
    public static String updateRobotPosition(Pose2d pose) {
        //Adds roadrunner pose data to string, rounded to nearest 2 decimal places
        return "x: " + (double)Math.round(pose.getX()*100)/100 + " y: " + (double)Math.round(pose.getY()*100)/100 +
                " heading: " + (double)Math.round(pose.getHeading()*180/Math.PI*100)/100; //Heading is converted from radians to degrees
    }

    @SuppressLint("DefaultLocale")
    public static String updateRobotPosAprilTag(AprilTagDetection aprilTag) {
        //Adding all april tag pose data to a string, numbers are rounded to nearest 2 decimal places
        return String.format(" x: %,3.2f", aprilTag.ftcPose.x) + String.format(" y: %,3.2f", aprilTag.ftcPose.y) + String.format(" z: %,3.2f", aprilTag.ftcPose.z) + String.format(" heading: %,3.2f", aprilTag.ftcPose.yaw) + String.format(" confidence: %,3.2f", aprilTag.decisionMargin);
    }

    public static void outtakePlace (HwMap hwMap) {
        ElapsedTime dropTime = new ElapsedTime();
        hwMap.dropServo.setPosition(RobotConstants.dropOpen);
        dropTime.reset();
        if (dropTime.seconds() > RobotConstants.dropTime) {
            hwMap.dropServo.setPosition(RobotConstants.dropClosed);
        }
    }

    /*public void setTargetPos(int distLeft) {
        accelTime = maxVelo / maxAccel;
        halfDist = distLeft / 2;
        accelDist = .5 * maxAccel * Math.pow(accelTime, 2);

        if (accelDist > halfDist) {
            accelDist = Math.sqrt(halfDist / (.5 * maxAccel));
        }
        accelDist = .5 * maxAccel * Math.pow(accelTime, 2);

        double newMaxVelo =  maxAccel * accelDist; // changes max velo based on max accel and dist to accel

        decelDist = accelDist;
        constDist = distLeft - (accelDist * 2);
        constTime = constDist / maxVelo;
        decelTime = accelTime + constTime;
    }*/

    /*public double slidesUpdate(double maxAccel, double maxVelo, int finalPos) {
        ElapsedTime timer = new ElapsedTime();
        double constCurrTime;
        setTargetPos(finalPos);
        double totalTime = accelTime + constTime + decelTime;

        if (timer.seconds() > totalTime)
            return finalPos;

        if (timer.seconds() < accelTime) {
            return .5 * maxAccel * Math.pow(accelTime, 2);
        }
        else if (timer.seconds() < decelTime) {
            accelDist = .5 * maxAccel * Math.pow(accelTime, 2);
            constCurrTime = timer.seconds() - accelTime;
            return accelDist + maxVelo * constCurrTime;
        }
        else {
            accelDist = .5 * maxAccel * Math.pow(accelTime, 2);
            constDist = maxVelo * constTime;
            decelTime = timer.seconds() - decelTime;

            return accelDist + constDist + maxVelo * decelTime - .5 * maxAccel * Math.pow(accelTime, 2);
        }
    }*/

    public void setTargetPos(int startPos, int targetPos) {
        //Return the current reference position based on the given motion profile times, maximum acceleration, velocity, and current time.
        max_velocity = maxVelo;

        int distance = targetPos - startPos;
        startPosStore = startPos;
        targetPosStore = targetPos;
        // calculate the time it takes to accelerate to max velocity
        acceleration_dt = max_velocity / maxAccel;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfway_distance = distance / 2;
        acceleration_distance = 0.5 * maxAccel * Math.pow(acceleration_dt, 2);

        if (acceleration_distance > halfway_distance)
            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * maxAccel));

        acceleration_distance = 0.5 * maxAccel * Math.pow(acceleration_dt, 2);

        // recalculate max velocity based on the time we have to accelerate and decelerate
        max_velocity = maxAccel * acceleration_dt;

        // we decelerate at the same rate as we accelerate
        deceleration_dt = acceleration_dt;

        // calculate the time that we're at max velocity
        cruise_distance = distance - 2 * acceleration_distance;
        cruise_dt = cruise_distance / max_velocity;
        deceleration_time = acceleration_dt + cruise_dt;

        elapsed_time.reset();
    }


    // check if we're still in the motion profile
    public double slidesUpdate() {
        double entire_dt = acceleration_dt + cruise_dt + deceleration_dt;
        if (elapsed_time.seconds() > entire_dt)
            return startPosStore;

        // if we're accelerating
        if (elapsed_time.seconds() < acceleration_dt)
            // use the kinematic equation for acceleration
            return startPosStore + 0.5 * RobotConstants.maxAccel * Math.pow(elapsed_time.seconds(), 2);

        // if we're cruising
        else if (elapsed_time.seconds() < deceleration_time) {
            acceleration_distance = 0.5 * RobotConstants.maxAccel * Math.pow(acceleration_dt, 2);
            double cruise_current_dt = elapsed_time.seconds() - acceleration_dt;

            // use the kinematic equation for constant velocity
            return startPosStore + acceleration_distance + max_velocity * cruise_current_dt;
        }

        // if we're decelerating
        else {
            acceleration_distance = 0.5 * maxAccel * Math.pow(acceleration_dt, 2);
            cruise_distance = max_velocity * cruise_dt;
            deceleration_time = elapsed_time.seconds() - deceleration_time;

            // use the kinematic equations to calculate the instantaneous desired position
            return startPosStore + cruise_distance + max_velocity * deceleration_time - 0.5 * maxAccel * Math.pow(deceleration_time, 2);
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
