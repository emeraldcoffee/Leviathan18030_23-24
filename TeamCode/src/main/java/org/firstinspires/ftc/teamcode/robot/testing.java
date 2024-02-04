package org.firstinspires.ftc.teamcode.robot;

import android.annotation.SuppressLint;
import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.BackDropLocalizer;
import org.firstinspires.ftc.teamcode.drive.RearWallLocalizer;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class testing extends LinearOpMode {

    public WebcamName frontCamera;
    public Encoder rightEncoder;

    @SuppressLint("DefaultLocale")
    public void runOpMode() {
        //Init code
//        HwMap robot = new HwMap();
//        robot.init(hardwareMap);
        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);
        BackDropLocalizer backDropLocalizer = new BackDropLocalizer(hardwareMap);
        RearWallLocalizer rearWallLocalizer = new RearWallLocalizer(hardwareMap);

//        frontCamera = hardwareMap.get(WebcamName.class, "camera");



        ElapsedTime runTime = new ElapsedTime();
//        Telemetry.Item MotorCurrents = telemetry.addData("Motor Currents", "");
//        Telemetry.Item frontPoseEstimate = telemetry.addData("Front Pose Estimate", "");
//        Telemetry.Item rearPoseEstimate = telemetry.addData("Rear Pose Estimate", "");
        Telemetry.Item encoderVal = telemetry.addData("Encoder val", "");
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "transferMotor"));


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
//            MotorCurrents.setValue(String.format("transfer motor: %,3.2f intake motor %,3.2f", robot.transferMotor.getCurrent(CurrentUnit.AMPS), robot.intakeMotor.getCurrent(CurrentUnit.AMPS)));
//            backDropLocalizer.update();
//            rearWallLocalizer.update();
//
//            frontPoseEstimate.setValue(RobotMethods.updateRobotPosition(backDropLocalizer.getPoseEstimate()));
//
//            rearPoseEstimate.setValue(RobotMethods.updateRobotPosition(rearWallLocalizer.getPoseEstimate()));
//
//            driveTrain.setPoseEstimate(backDropLocalizer.getPoseEstimate());
//            driveTrain.update();
//

            encoderVal.setValue(rightEncoder.getCurrentPosition());
//            sleep(15);

            telemetry.update();
        }

    }


}
