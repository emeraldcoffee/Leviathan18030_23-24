package org.firstinspires.ftc.teamcode.robot;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.RobotConfig;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.i2cDrivers.UltraSonic;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.ArrayList;
import java.util.List;


@TeleOp
public class testing extends LinearOpMode {



    @SuppressLint("DefaultLocale")
    public void runOpMode() {
        //Init code
        RobotConfig robot = new RobotConfig(hardwareMap);

        ElapsedTime runTime = new ElapsedTime();

        ElapsedTime readingTimer = new ElapsedTime();

        Telemetry.Item loopSpeed = telemetry.addData("Loop Speed", "");

        Telemetry.Item distance = telemetry.addData("Backdrop estimate", "");
        Telemetry.Item poseEstimate = telemetry.addData("Robot estimate", "");
        Telemetry.Item isReading = telemetry.addData("isReading", "");


        Gamepad currentGamepad1 = new Gamepad();
        Gamepad prevGamepad1 = new Gamepad();



        waitForStart();
        if (isStopRequested()) return;
        robot.setPoseEstimate(new Pose2d(16.7, -62, Math.toRadians(90)));

        robot.stackHold(false);
        robot.stackArm(RobotConfig.StackArm.OUT);


        while (opModeIsActive() && !isStopRequested()) {
            currentGamepad1.copy(gamepad1);

            if (readingTimer.seconds()>1) {
                robot.updateBackdropLocalizer();
                Pose2d leftPose = robot.getPoseEstimateRight();
                robot.takeRightReading();
                readingTimer.reset();
                distance.setValue(String.format("x: %,3.2f, y: %,3.2f, heading: %,3.2f", leftPose.getX(), leftPose.getY(), leftPose.getHeading()));
            }

            isReading.setValue(robot.isRightReading());

            robot.update();
            poseEstimate.setValue(String.format("x: %,3.2f, y: %,3.2f, heading: %,3.2f", robot.getPoseEstimate().getX(), robot.getPoseEstimate().getY(), robot.getPoseEstimate().getHeading()));

            prevGamepad1.copy(currentGamepad1);

            loopSpeed.setValue(runTime.milliseconds());
            runTime.reset();
            telemetry.update();
            }
        }

    }
