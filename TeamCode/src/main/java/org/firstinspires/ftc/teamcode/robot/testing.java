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

//@Disabled
@TeleOp
public class testing extends LinearOpMode {



    @SuppressLint("DefaultLocale")
    public void runOpMode() {
        //Init code
        RobotConfig robot = new RobotConfig(hardwareMap);


        ElapsedTime runTime = new ElapsedTime();

        Telemetry.Item loopSpeed = telemetry.addData("Loop Speed", "");

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad prevGamepad1 = new Gamepad();


        waitForStart();
        if (isStopRequested()) return;

        robot.stackHold(false);
        robot.stackArm(RobotConfig.StackArm.OUT);

        while (opModeIsActive() && !isStopRequested()) {
            currentGamepad1.copy(gamepad1);

            if (currentGamepad1.left_bumper && !prevGamepad1.left_bumper) {
                robot.grabFromStack(2);
            } else if (currentGamepad1.right_bumper && !prevGamepad1.right_bumper) {
                robot.grabFromStack(1);
            }

            if (currentGamepad1.dpad_down && !prevGamepad1.dpad_down) {
                robot.intakeMotor.setPower(1);
                robot.transferMotor.setPower(1);
            } else if (!currentGamepad1.dpad_down && prevGamepad1.dpad_down) {
                robot.intakeMotor.setPower(0);
                robot.transferMotor.setPower(0);
            }


            robot.update();

            prevGamepad1.copy(currentGamepad1);

            loopSpeed.setValue(runTime.milliseconds());
            runTime.reset();
            telemetry.update();
            }
        }

    }
