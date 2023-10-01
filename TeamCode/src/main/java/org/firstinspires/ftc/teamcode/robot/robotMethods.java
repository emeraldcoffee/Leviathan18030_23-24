package org.firstinspires.ftc.teamcode.robot;

import static java.lang.Math.abs;
import static java.lang.Math.max;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class robotMethods {

    //Takes input values and sets drivetrain to corresponding powers while scaling all powers under 1
    public static void setMecanumDrive(double forward, double strafe, double turn, SampleMecanumDrive driveTrain) {
        //Find value to make all motor powers less than 1
        double scalePower = max(abs(forward) + abs(strafe) + abs(turn), 1);

        //Creating string with all drive powers for mecanum drive
        Double[] driveSpeeds = {forward+strafe+turn/scalePower, forward-strafe+turn/scalePower, forward-strafe-turn/scalePower, forward+strafe-turn/scalePower};

        //Setting motors to there new powers
        driveTrain.setMotorPowers(driveSpeeds[0], driveSpeeds[1], driveSpeeds[2], driveSpeeds[3]);
    }

    //Takes input values and sets drivetrain to corresponding powers while scaling all powers under maxPower
    public static void setMecanumDrive(double forward, double strafe, double turn, double maxPower, SampleMecanumDrive driveTrain) {
        //Find value to make all motor powers less than 1
        double scalePower = max(abs(forward) + abs(strafe) + abs(turn), maxPower);

        //Creating string with all drive powers for mecanum drive
        Double[] driveSpeeds = {(forward+strafe+turn)/scalePower, (forward-strafe+turn)/scalePower,
                                (forward-strafe-turn/scalePower), (forward+strafe-turn)/scalePower};

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


}
