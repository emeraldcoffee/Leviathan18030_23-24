package org.firstinspires.ftc.teamcode.robot;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Timer;

@TeleOp
public class DistanceSensorLocalization extends LinearOpMode {

    @SuppressLint("DefaultLocale")
    public void runOpMode() {
        HwMap robot = new HwMap();
        robot.init(hardwareMap);
        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);

        Telemetry.Item distance = telemetry.addData("Left Distance", "");
        Telemetry.Item poseEstimate = telemetry.addData("Lateral Distance", "");
        Telemetry.Item loopSpeed = telemetry.addData("Loop Speed", "");

        ElapsedTime loopTimer = new ElapsedTime();

        waitForStart();
        loopTimer.reset();

        while (opModeIsActive() && !isStopRequested()) {
//13.5-1.11909-2.64
            Pose2d currentPose = driveTrain.getPoseEstimate();

            double leftDistance = robot.leftDistanceSensor.getDistance(DistanceUnit.INCH);
            double rightDistance = robot.rightDistanceSensor.getDistance(DistanceUnit.INCH);

            distance.setValue(String.format( "%,3.2f", leftDistance) + String.format( " Right Distance: %,3.2f", rightDistance));

            double heading = -Math.tan((leftDistance-rightDistance)/10.03);
            double lateralDistance = ((leftDistance+rightDistance)/2+9.74)*Math.cos(heading);

            poseEstimate = poseEstimate.setValue(String.format("%,3.2f", lateralDistance) + String.format(" Heading %,3.2f", heading*180/Math.PI));

            driveTrain.setPoseEstimate(new Pose2d(lateralDistance-70, currentPose.getY(), heading));

            driveTrain.update();
            telemetry.update();

            loopSpeed.setValue(loopTimer.milliseconds());
            loopTimer.reset();
        }


    }
}
