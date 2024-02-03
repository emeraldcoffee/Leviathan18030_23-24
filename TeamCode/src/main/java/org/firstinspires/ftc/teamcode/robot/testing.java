package org.firstinspires.ftc.teamcode.robot;

import android.annotation.SuppressLint;
import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.BackDropLocalizer;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class testing extends LinearOpMode {

    public WebcamName frontCamera;

    @SuppressLint("DefaultLocale")
    public void runOpMode() {
        //Init code
//        HwMap robot = new HwMap();
//        robot.init(hardwareMap);
//        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);
        BackDropLocalizer backDropLocalizer = new BackDropLocalizer(hardwareMap);

//        frontCamera = hardwareMap.get(WebcamName.class, "camera");



        ElapsedTime runTime = new ElapsedTime();
//        Telemetry.Item MotorCurrents = telemetry.addData("Motor Currents", "");
        Telemetry.Item poseEstimate = telemetry.addData("Pose Estimate", "");

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
//            MotorCurrents.setValue(String.format("transfer motor: %,3.2f intake motor %,3.2f", robot.transferMotor.getCurrent(CurrentUnit.AMPS), robot.intakeMotor.getCurrent(CurrentUnit.AMPS)));
            backDropLocalizer.update();

            poseEstimate.setValue(RobotMethods.updateRobotPosition(backDropLocalizer.getPoseEstimate()));
            sleep(15);

            telemetry.update();
        }

    }


}
