package org.firstinspires.ftc.teamcode.robot;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.i2cDrivers.UltraSonic;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class testing extends LinearOpMode {

    public WebcamName frontCamera;
    public Encoder rightEncoder;

    public UltraSonic leftUltraSonic;

    enum ReadCycle {
        START_READ,
        WAIT_FOR_READ,
        READ

    }

    enum TestAlign {
        WAIT,
        DRIVE_INFRONT,
        DRIVE_TO
    }

    TestAlign testAlign = TestAlign.WAIT;

    ReadCycle readCycle = ReadCycle.START_READ;

    @SuppressLint("DefaultLocale")
    public void runOpMode() {
        //Init code
//        HwMap robot = new HwMap();
//        robot.init(hardwareMap);
        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);
        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();


//        frontCamera = hardwareMap.get(WebcamName.class, "camera");



        ElapsedTime runTime = new ElapsedTime();
//        Telemetry.Item MotorCurrents = telemetry.addData("Motor Currents", "");
//        Telemetry.Item frontPoseEstimate = telemetry.addData("Front Pose Estimate", "");
//        Telemetry.Item rearPoseEstimate = telemetry.addData("Rear Pose Estimate", "");
        Telemetry.Item poseEstimate = telemetry.addData("Pose Estimate", "");
//        Telemetry.Item ultraSonicVal = telemetry.addData("UltraSonic:", "");
//
//        leftUltraSonic = hardwareMap.get(UltraSonic.class, "leftUltraSonic");
        Telemetry.Item loopSpeed = telemetry.addData("Loop Speed", "");

        ElapsedTime timer = new ElapsedTime();


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            driveTrain.update();

            switch (testAlign) {
                case WAIT:
                    if (gamepad1.a) {
                        driveTrain.updateBackdrop();
                        TrajectorySequence test = driveTrain.trajectorySequenceBuilder(driveTrain.getPoseEstimate())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, Math.toRadians(310), 10.62))
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(20))
                                .lineToSplineHeading(new Pose2d(50, driveTrain.getPoseEstimate().getY(), Math.toRadians(0)))
                                .waitSeconds(.2)
                                .build();
                        driveTrain.followTrajectorySequenceAsync(test);
                        testAlign = TestAlign.DRIVE_INFRONT;
                    }
                    break;
                case DRIVE_INFRONT:
                    if (!driveTrain.isBusy()) {
                        TrajectorySequence test = driveTrain.trajectorySequenceBuilder(driveTrain.getPoseEstimate())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(310), 10.62))
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(30))
                                .lineToSplineHeading(new Pose2d(52, driveTrain.getPoseEstimate().getY(), Math.toRadians(0)))
                                .build();
                        driveTrain.followTrajectorySequenceAsync(test);
                        testAlign = TestAlign.DRIVE_TO;
                    } else driveTrain.compensatedUpdateBackdrop();

                    break;
                case DRIVE_TO:
                    if (!driveTrain.isBusy()) {
                        testAlign = TestAlign.WAIT;
                    }
                    break;
            }

            loopSpeed.setValue(runTime.milliseconds());

            runTime.reset();

            telemetry.update();
        }

    }
//   switch (readCycle) {
////                case START_READ:
////                    leftUltraSonic.rangeReading();
////                    timer.reset();
////                    readCycle = ReadCycle.WAIT_FOR_READ;
////                    break;
////                case WAIT_FOR_READ:
////                    if (timer.milliseconds() > 100) {
////                        readCycle = ReadCycle.READ;
////                    }
////                    break;
////                case READ:
////                    ultraSonicVal.setValue(leftUltraSonic.reportRangeReadingIN());
////                    readCycle = ReadCycle.START_READ;
////            }
//
//
//            frontPoseEstimate.setValue(RobotMethods.updateRobotPosition(backDropLocalizer.getPoseEstimate()));
////            rearPoseEstimate.setValue(RobotMethods.updateRobotPosition(rearWallLocalizer.getPoseEstimate()));
//            poseEstimate.setValue(RobotMethods.updateRobotPosition(driveTrain.getPoseEstimate()));
//
////            rearWallLocalizer.update();
//            backDropLocalizer.update();
//            driveTrain.update();
//
//            loopSpeed.setValue(runTime.milliseconds());
//            runTime.reset();

}
