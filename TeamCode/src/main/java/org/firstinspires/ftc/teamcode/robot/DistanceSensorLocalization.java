package org.firstinspires.ftc.teamcode.robot;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.BackDropLocalizer;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Timer;

@Disabled
@TeleOp
public class DistanceSensorLocalization extends LinearOpMode {

    @SuppressLint("DefaultLocale")
    public void runOpMode() {
        HwMap robot = new HwMap();
        robot.init(hardwareMap);
        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);
        BackDropLocalizer backDropLocalizer = new BackDropLocalizer(hardwareMap);

        Telemetry.Item distance = telemetry.addData("Left Distance", "");
        Telemetry.Item poseEstimate = telemetry.addData("Lateral Distance", "");
        Telemetry.Item loopSpeed = telemetry.addData("Loop Speed", "");

        ElapsedTime loopTimer = new ElapsedTime();

        waitForStart();
        loopTimer.reset();

        while (opModeIsActive() && !isStopRequested()) {

            backDropLocalizer.update();
            poseEstimate = poseEstimate.setValue(String.format("%,3.2f Heading: %,3.2f", backDropLocalizer.getPoseEstimate().getX(), Math.toDegrees(backDropLocalizer.getPoseEstimate().getHeading())));


            driveTrain.update();
            telemetry.update();

            loopSpeed.setValue(loopTimer.milliseconds());
            loopTimer.reset();
        }


    }
}
