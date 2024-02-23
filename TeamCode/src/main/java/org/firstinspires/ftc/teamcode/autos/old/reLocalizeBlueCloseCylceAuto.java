package org.firstinspires.ftc.teamcode.autos.old;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.RobotConfig;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.ColorMask;
import org.firstinspires.ftc.teamcode.robot.HwMap;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.testing;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous(name = "reLocalize Blue Close Auto")
public class reLocalizeBlueCloseCylceAuto extends LinearOpMode {

    enum Camera {
        WAIT,
        SAVE,
        FINISHED
    }
    Camera camera = Camera.WAIT;

    enum BackDropAlign {
        WAIT,
        START,
        DRIVE_INFRONT,
        DRIVE_TO
    }
    BackDropAlign backDropAlign = BackDropAlign.WAIT;

    double slideI = 0;

    String pos = "";

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {

//        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);
//        HwMap robot = new HwMap();
//        robot.init(hardwareMap);

        RobotConfig robot = new RobotConfig(hardwareMap);
        ColorMask pipeline = new ColorMask();




        robot.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            public void onOpened()
            {
                pipeline.setAlliance("Blue");
                robot.webcam.setPipeline(pipeline);

                robot.webcam.startStreaming(640,480, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error: ", errorCode);
            }
        });


        robot.transferMotor.setPower(-.2);
        robot.stackHold(false);
        robot.dropServo.setPosition(RobotConstants.dropClosed);

        //8.29 sec
        TrajectorySequence cycle = robot.trajectorySequenceBuilder(new Pose2d(52, 30, Math.toRadians(0)))
                .waitSeconds(.2)
                .back(.1)
                .splineToConstantHeading(new Vector2d(22, 11), Math.toRadians(180))
                .addTemporalMarker(1, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                    robot.dropServo.setPosition(RobotConstants.dropClosed);
                })
                .lineTo(new Vector2d(-30, 11))
                .addTemporalMarker(2.9, () -> {
                    robot.transferMotor.setPower(1);
                    robot.intakeMotor.setPower(1);
                    robot.stackArm(RobotConfig.StackArm.OUT);
                })
                .splineToConstantHeading(new Vector2d(-55, 12), Math.toRadians(180))
                .waitSeconds(.9)
                .addTemporalMarker(3.8, () -> {
                    robot.stackHold(true);
                })
                .addTemporalMarker(3.9, () -> {
                    robot.stackArm(RobotConfig.StackArm.IN);
                })
                .addTemporalMarker(4.3, () -> {
                    robot.stackArm(RobotConfig.StackArm.OUT);
                })
                .addTemporalMarker(4.7, () -> {
                    robot.stackArm(RobotConfig.StackArm.IN);
                })
                .forward(.1)
                .splineToConstantHeading(new Vector2d(-30, 11), Math.toRadians(0))
                .addTemporalMarker(6.9, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.LOW);
                    robot.transferMotor.setPower(0);
                    robot.intakeMotor.setPower(0);
                })
                .lineTo(new Vector2d(22, 11))
                .splineToConstantHeading(new Vector2d(49.5, 30), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                                backDropAlign = BackDropAlign.START;
                })
                .build();

        //.63 sec
        TrajectorySequence end = robot.trajectorySequenceBuilder(cycle.start())
                .back(5)
                .addTemporalMarker(.3, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
//                    targetSlidePos = RobotConstants.slideBottom;
                })
                .build();

        //12.69 sec
        TrajectorySequence left = robot.trajectorySequenceBuilder(new Pose2d(13.4, 64, Math.toRadians(270)))
                .forward(3)
                .splineToLinearHeading(new Pose2d(23, 42), Math.toRadians(270))
                .lineTo(new Vector2d(23, 46))
                .addTemporalMarker(1.7, () -> {
                    robot.rightPixelServo.setPosition(RobotConstants.rightIn);
                })
                .lineTo(new Vector2d(26, 46))
                .splineToConstantHeading(new Vector2d(45, 41), Math.toRadians(0))

                .lineTo(new Vector2d(51.7, 41))
                .waitSeconds(.3)
                .addTemporalMarker(4.1, () -> {
                    robot.dropServo.setPosition(RobotConstants.dropOpen);
                    robot.safeRelocalizeBackdrop();
                })
                .back(.1)
                .splineToConstantHeading(new Vector2d(22, 11), Math.toRadians(180))
                .addTemporalMarker(5.2, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                    robot.dropper(RobotConfig.Dropper.CLOSED);
                })
                .lineTo(new Vector2d(-30, 11))
                .addTemporalMarker(5.5, () -> {
                    robot.transferMotor.setPower(.2);
                })
                .addTemporalMarker(7.0, () -> {
                    robot.transferMotor.setPower(1);
                    robot.intakeMotor.setPower(1);
                    robot.stackArm(RobotConfig.StackArm.OUT);
                })
                .splineToConstantHeading(new Vector2d(-55, 12), Math.toRadians(180))
                .waitSeconds(.9)
                .addTemporalMarker(8.2, () -> {
                    robot.stackHold(true);
                })
                .addTemporalMarker(8.3, () -> {
                    robot.stackArm(RobotConfig.StackArm.IN);
                })
                .addTemporalMarker(8.7, () -> {
                    robot.stackArm(RobotConfig.StackArm.OUT);
                })
                .addTemporalMarker(9.1, () -> {
                    robot.stackArm(RobotConfig.StackArm.IN);
                })
                .forward(.1)
                .splineToConstantHeading(new Vector2d(-30, 11), Math.toRadians(0))
                .addTemporalMarker(11.1, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.LOW);
                    robot.transferMotor.setPower(0);
                    robot.intakeMotor.setPower(0);
                })
                .lineTo(new Vector2d(22, 11))
                .splineToConstantHeading(new Vector2d(49.5, 30), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    backDropAlign = BackDropAlign.START;
                })
                .build();

        TrajectorySequence center = robot.trajectorySequenceBuilder(new Pose2d(12, 63, Math.toRadians(270)))
                .lineTo(new Vector2d(12, 60))
                .splineToSplineHeading(new Pose2d(17, 34), Math.toRadians(270))
                .lineTo(new Vector2d(17,37))
//                .addTemporalMarker(1.7, () -> robot.rightServo.setPosition(RobotConstants.rightIn))
                .lineTo(new Vector2d(18, 37))
                .splineToConstantHeading(new Vector2d(45, 37.5), Math.toRadians(0))
                .lineTo(new Vector2d(53, 37.5))
//                .addTemporalMarker(3.6, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
                .waitSeconds(.3)
                .lineTo(new Vector2d(40, 37.5))
//                .addDisplacementMarker(() -> {targetSlidePos = RobotConstants.slideBottom; robot.dropServo.setPosition(RobotConstants.dropClosed);})
                .lineTo(new Vector2d(40, 60))
                .lineTo(new Vector2d(45, 60))
                .build();

        TrajectorySequence right = robot.trajectorySequenceBuilder(new Pose2d(12, 63, Math.toRadians(270)))
                .lineTo(new Vector2d(12, 45))
                .splineToConstantHeading(new Vector2d(8.5, 36), Math.toRadians(180))
                .lineTo(new Vector2d(7, 36))
//                .addTemporalMarker(1.6, () -> robot.rightServo.setPosition(RobotConstants.rightIn))
                .lineTo(new Vector2d(20, 36))
                .splineToSplineHeading(new Pose2d(45, 28.5, Math.toRadians(0)), Math.toRadians(0))
                .lineTo(new Vector2d(53, 28.5))
//                .addTemporalMarker(3.7, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
                .waitSeconds(.3)
                .lineTo(new Vector2d(40, 28.5))
//                .addDisplacementMarker(() -> {targetSlidePos = RobotConstants.slideBottom; robot.dropServo.setPosition(RobotConstants.dropClosed);})
                .lineTo(new Vector2d(40, 60))
                .lineTo(new Vector2d(45, 60))
                .build();

        ElapsedTime cameraDelayTimer = new ElapsedTime();
        ElapsedTime runTime = new ElapsedTime();
        ElapsedTime loopSpeedTimer = new ElapsedTime();

        telemetry.setAutoClear(false);
        Telemetry.Item detectedPos = telemetry.addData("Position", "No detection");
        Telemetry.Item loopSpeed = telemetry.addData("Loop Speed", "");
        Telemetry.Item imu = telemetry.addData("imu", robot.getPoseEstimate().getHeading());
//        Telemetry.Item wasPos = telemetry.addData("Was pos", "");
//        Telemetry.Item slideData = telemetry.addData("Slide Data:", "Encoder Val:" + robot.liftEncoder.getCurrentPosition() + " Target Val:" + targetSlidePos);

        robot.rightPixelServo.setPosition(RobotConstants.rightOut);

        waitForStart();
        if (isStopRequested()) return;

        runTime.reset();
        loopSpeedTimer.reset();

        robot.setTargetSlidePos(RobotConfig.SlideHeight.PRELOAD_DROP);

        robot.update();
        robot.setPoseEstimate(new Pose2d(13.4, 64, Math.toRadians(270)));

        cameraDelayTimer.reset();

        while (opModeIsActive() && !isStopRequested()) {

            switch (camera) {
                case WAIT:
                    if (cameraDelayTimer.seconds() > .8) {
                        camera = Camera.SAVE;
                    }
                    break;
                case SAVE:
                    pos = pipeline.getPos();
                    switch ("left") {
                        case "left":
                            robot.followTrajectorySequenceAsync(left);
                            detectedPos.setValue("Left");
                            break;
                        case "center":
                            robot.followTrajectorySequenceAsync(center);
                            detectedPos.setValue("Center");
                            break;
                        case "right":
                            robot.followTrajectorySequenceAsync(right);
                            detectedPos.setValue("Right");
                            break;
                        default:
                            robot.followTrajectorySequenceAsync(center);
                            detectedPos.setValue("Default center (No detection)");
                            break;

                    }

                    camera = Camera.FINISHED;
                    break;
                case FINISHED:
                    break;
            }

            switch (backDropAlign) {
                case START:
                    robot.safeRelocalizeBackdrop();
                    TrajectorySequence driveInfront = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                            .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, Math.toRadians(310), 10.62))
                            .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(20))
                            .lineToSplineHeading(cycle.start().minus(new Pose2d(1.5, 0, 0)))
                            .waitSeconds(.2)
                            .build();
                    robot.followTrajectorySequenceAsync(driveInfront);

                    backDropAlign = BackDropAlign.DRIVE_INFRONT;
                    break;
                case DRIVE_INFRONT:
                    if (!robot.isBusy()) {
                        TrajectorySequence driveTo = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(310), 10.62))
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(30))
                                .lineToSplineHeading(new Pose2d(52, cycle.start().getY(), Math.toRadians(0)))
                                .build();
                        robot.followTrajectorySequenceAsync(driveTo);
                        backDropAlign = BackDropAlign.DRIVE_TO;
                    } else robot.safeRelocalizeBackdrop();
                    break;
                case DRIVE_TO:
                    if (!robot.isBusy()) {
                        robot.dropper(RobotConfig.Dropper.PARTIAL);
                        backDropAlign = BackDropAlign.WAIT;
                        //Checks if there is time to run a cycle
                        if (runTime.seconds()<(30-cycle.duration()-end.duration())) {
//                            driveTrain.followTrajectorySequenceAsync(cycle);
                        } else {
//                            driveTrain.followTrajectorySequenceAsync(end);
                        }
                    }
                    break;
            }
            robot.update();


            loopSpeed.setValue(String.format("%,3.2f ms", loopSpeedTimer.milliseconds()));
            loopSpeedTimer.reset();

            imu.setValue(robot.getPoseEstimate().getHeading());

            telemetry.update();
        }

    }
}