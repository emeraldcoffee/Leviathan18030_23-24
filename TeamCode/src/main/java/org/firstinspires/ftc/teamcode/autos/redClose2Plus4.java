package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.RobotConfig;
import org.firstinspires.ftc.teamcode.pipelines.ColorMask;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous
public class redClose2Plus4 extends LinearOpMode {
    enum Camera {
        WAIT,
        SAVE,
        FINISHED
    }

    Camera camera = Camera.WAIT;

    String pos = "";

    @Override
    public void runOpMode() throws InterruptedException {

        RobotConfig robot = new RobotConfig(hardwareMap);
        ColorMask pipeline = new ColorMask();

        robot.ResetSlides();

        robot.transferMotor.setPower(-.2);
//        robot.spikeMarkHoldServo.setPosition(RobotConstants.holdServoUp);
        robot.stackHold(false);

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        robot.webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"));

        robot.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                pipeline.setAlliance("Red");
                robot.webcam.setPipeline(pipeline);

                robot.webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error: ", errorCode);
            }
        });

        //4.57 stack 1, 15.09 stack 2
        TrajectorySequence right = robot.trajectorySequenceBuilder(new Pose2d(12, -62, Math.toRadians(90)))
                .lineTo(new Vector2d(12, -61))
                .splineToLinearHeading(new Pose2d(23, -40), Math.toRadians(90))
                .lineTo(new Vector2d(23, -48))
                .addTemporalMarker(2.0, () -> robot.leftPixelServo.setPosition(RobotConstants.leftIn))
                .lineTo(new Vector2d(26, -48))
                .splineToConstantHeading(new Vector2d(47, -44), Math.toRadians(0))
                .lineTo(new Vector2d(53, -44))
                .addTemporalMarker(4.5, () -> {
                    robot.dropper(RobotConfig.Dropper.OPEN);
                    robot.safeRelocalizeBackdrop();
                })
                .waitSeconds(.4)
                //cycle 1
                .lineTo(new Vector2d(50, -44))
                .addTemporalMarker(5.5, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                    robot.dropper(RobotConfig.Dropper.CLOSED);
                })
                .addTemporalMarker(8.0, () -> {
                    robot.intakeMotor.setPower(1);
                    robot.transferMotor.setPower(1);
                    robot.stackArm(RobotConfig.StackArm.OUT);
                })
                .splineToConstantHeading(new Vector2d(38, -15), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(35, -7), Math.toRadians(180))
                .lineTo(new Vector2d(-50, -7))
                .splineToConstantHeading(new Vector2d(-57, -11), Math.toRadians(180))
                .waitSeconds(.9)
                .addTemporalMarker(9.2, () -> {
                    robot.grabFromStack(1);
                })
                .splineToConstantHeading(new Vector2d(-50, -7), Math.toRadians(0))
                .lineTo(new Vector2d(30, -7))
                .addTemporalMarker(12.0, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.LOW);
                    robot.intakeMotor.setPower(0);
                    robot.transferMotor.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(50, -44), Math.toRadians(0))
                .lineTo(new Vector2d(53, -44))
                .waitSeconds(1.3)
                .back(.2)
                .addTemporalMarker(14.5, () -> {
                    robot.dropper(RobotConfig.Dropper.PARTIAL);
                    robot.safeRelocalizeBackdrop();
                })

                // cycle 2
                .lineTo(new Vector2d(50, -44))
                .splineToConstantHeading(new Vector2d(38, -15), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(35, -7), Math.toRadians(180))
                .addTemporalMarker(16.0, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                    robot.dropper(RobotConfig.Dropper.CLOSED);
                })
                .lineTo(new Vector2d(-50, -7))
                .addTemporalMarker(18.6, () -> {
                    robot.intakeMotor.setPower(1);
                    robot.transferMotor.setPower(1);
                    robot.stackArm(RobotConfig.StackArm.OUT);
                })
                .splineToConstantHeading(new Vector2d(-57, -11), Math.toRadians(180))
                .waitSeconds(1.7)
                .addTemporalMarker(19.7, () -> {
                    robot.grabFromStack(2);
                })
                .splineToConstantHeading(new Vector2d(-50, -7), Math.toRadians(0))
                .lineTo(new Vector2d(30, -7))
                .addTemporalMarker(23.5, () -> {
                    robot.setTargetSlidePos(18);
                    robot.intakeMotor.setPower(0);
                    robot.transferMotor.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(50, -44), Math.toRadians(0))
                .lineTo(new Vector2d(53, -44))
                .waitSeconds(1.3)
                .back(6)
                .addTemporalMarker(26.3, () -> {
                    robot.dropper(RobotConfig.Dropper.PARTIAL);
                    robot.safeRelocalizeBackdrop();
                })
                .addTemporalMarker(28.0, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                    robot.dropper(RobotConfig.Dropper.CLOSED);
                })
                .waitSeconds(2)
                .build();

        //3.88 stack 1, 14.68 stack 2
        TrajectorySequence center = robot.trajectorySequenceBuilder(new Pose2d(12, -62, Math.toRadians(90)))
                .lineTo(new Vector2d(12, -60))
                .splineToSplineHeading(new Pose2d(17, -31), Math.toRadians(90))
                .lineTo(new Vector2d(17,-36))
                .addTemporalMarker(1.8, () -> robot.leftPixelServo.setPosition(RobotConstants.leftIn))
                .lineTo(new Vector2d(18, -36))
                .splineToConstantHeading(new Vector2d(45, -36.5), Math.toRadians(0))
                .lineTo(new Vector2d(51.5, -36.5))
                .addTemporalMarker(4.25, () -> robot.dropper(RobotConfig.Dropper.OPEN))
                .waitSeconds(.3)
                .back(.1)
                .addTemporalMarker(5.2, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                    robot.dropper(RobotConfig.Dropper.CLOSED);
                    robot.transferMotor.setPower(.3);
                })
                //cycle 1
                .splineToConstantHeading(new Vector2d(24, -11.3), Math.toRadians(180))
                .addTemporalMarker(8.0, () -> {
                    robot.intakeMotor.setPower(1);
                    robot.transferMotor.setPower(1);
                    robot.stackArm(RobotConfig.StackArm.OUT);
                })
                .lineTo(new Vector2d(-57.5, -11.3))
                .waitSeconds(1)
                .addTemporalMarker(8.9, () -> {
                    robot.grabFromStack(2);
                })
                .lineTo(new Vector2d(24, -11.3))
                .addTemporalMarker(12.0, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.LOW);
                    robot.intakeMotor.setPower(0);
                    robot.transferMotor.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(50, -30), Math.toRadians(0))
                .lineTo(new Vector2d(51.2, -30))
                .waitSeconds(.5)
                .addTemporalMarker(14.3, () -> {
                    robot.dropper(RobotConfig.Dropper.PARTIAL);
//                    robot.safeRelocalizeBackdrop();
                })
                .back(.1)
                .addTemporalMarker(15.3, () -> {//8.5
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                    robot.dropper(RobotConfig.Dropper.CLOSED);
                })
                //cycle 2
                .splineToConstantHeading(new Vector2d(24, -11.6), Math.toRadians(180))
                .addTemporalMarker(17.5, () -> {
                    robot.intakeMotor.setPower(1);
                    robot.transferMotor.setPower(1);
                    robot.stackArm(RobotConfig.StackArm.OUT);
                })
                .lineTo(new Vector2d(-57, -11.6))
                .waitSeconds(1)
                .addTemporalMarker(18.4, () -> {
                    robot.grabFromStack(2);
                })
                .lineTo(new Vector2d(24, -11))
                .addTemporalMarker(21.7, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.LOW);
                    robot.intakeMotor.setPower(0);
                    robot.transferMotor.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(50, -30), Math.toRadians(0))
                .lineTo(new Vector2d(51, -30))
                .waitSeconds(.5)
                .back(8)
                .addTemporalMarker(23.4, () -> {
                    robot.dropper(RobotConfig.Dropper.PARTIAL);
//                    robot.safeRelocalizeBackdrop();
                })
                .build();

        //3.38 stack 1, 14.46 stack 2
        TrajectorySequence left = robot.trajectorySequenceBuilder(new Pose2d(12, -62, Math.toRadians(90)))
                .addTemporalMarker(.1, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                })
                .lineTo(new Vector2d(12, -45))
                .splineToConstantHeading(new Vector2d(8.5, -36), Math.toRadians(180))
                .lineTo(new Vector2d(8, -36))
                .addTemporalMarker(1.5, () -> robot.leftPixelServo.setPosition(RobotConstants.leftIn))
                .lineTo(new Vector2d(20, -36))
                .splineToSplineHeading(new Pose2d(45, -31, Math.toRadians(0)), Math.toRadians(0))
                .lineTo(new Vector2d(53, -31))
                .addTemporalMarker(4.1, () -> robot.dropper(RobotConfig.Dropper.OPEN))
                .waitSeconds(.3)
                .back(6)
                .addTemporalMarker(5.0, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                    robot.dropper(RobotConfig.Dropper.CLOSED);
                })
                //cycle 1
                .splineToConstantHeading(new Vector2d(38, -15), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(35, -7), Math.toRadians(180))
                .addTemporalMarker(7.4, () -> {
                    robot.intakeMotor.setPower(1);
                    robot.transferMotor.setPower(1);
                    robot.stackArm(RobotConfig.StackArm.OUT);
                })
                .lineTo(new Vector2d(-50, -7))
                .splineToConstantHeading(new Vector2d(-57, -11), Math.toRadians(180))
                .waitSeconds(.9)
                .addTemporalMarker(8.6, () -> {
                    robot.grabFromStack(1);
                })
                .splineToConstantHeading(new Vector2d(-50, -7), Math.toRadians(0))
                .lineTo(new Vector2d(30, -7))
                .addTemporalMarker(11.4, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.LOW);
                    robot.intakeMotor.setPower(0);
                    robot.transferMotor.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(50, -44), Math.toRadians(0))
                .lineTo(new Vector2d(53, -44))
                .waitSeconds(1.3)
                .back(.2)
                .addTemporalMarker(13.9, () -> {
                    robot.dropper(RobotConfig.Dropper.PARTIAL);
                    robot.safeRelocalizeBackdrop();
                })

                // cycle 2
                .lineTo(new Vector2d(50, -44))
                .splineToConstantHeading(new Vector2d(38, -15), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(35, -7), Math.toRadians(180))
                .addTemporalMarker(15.4, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                    robot.dropper(RobotConfig.Dropper.CLOSED);
                })
                .lineTo(new Vector2d(-50, -7))
                .addTemporalMarker(18.0, () -> {
                    robot.intakeMotor.setPower(1);
                    robot.transferMotor.setPower(1);
                    robot.stackArm(RobotConfig.StackArm.OUT);
                })
                .splineToConstantHeading(new Vector2d(-57, -11), Math.toRadians(180))
                .waitSeconds(1.7)
                .addTemporalMarker(19.1, () -> {
                    robot.grabFromStack(2);
                })
                .splineToConstantHeading(new Vector2d(-50, -7), Math.toRadians(0))
                .lineTo(new Vector2d(30, -7))
                .addTemporalMarker(22.9, () -> {
                    robot.setTargetSlidePos(18);
                    robot.intakeMotor.setPower(0);
                    robot.transferMotor.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(50, -44), Math.toRadians(0))
                .lineTo(new Vector2d(53, -44))
                .waitSeconds(1.3)
                .back(6)
                .addTemporalMarker(25.7, () -> {
                    robot.dropper(RobotConfig.Dropper.PARTIAL);
                    robot.safeRelocalizeBackdrop();
                })
                .addTemporalMarker(27.4, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                    robot.dropper(RobotConfig.Dropper.CLOSED);
                })
                .waitSeconds(1.5)
                .build();

        ElapsedTime cameraDelayTimer = new ElapsedTime();

        telemetry.setAutoClear(false);
        Telemetry.Item detectedPos = telemetry.addData("Position", "No detection");

//        Telemetry.Item slideData = telemetry.addData("Slide Data:", "Encoder Val:" + robot.liftEncoder.getCurrentPosition() + " Target Val:" + targetSlidePos);


        robot.leftPixelServo.setPosition(RobotConstants.leftOut);
        robot.dropper(RobotConfig.Dropper.CLOSED);

        robot.ResetSlides();

        waitForStart();
        if (isStopRequested()) return;

        robot.setTargetSlidePos(11);//RobotConfig.SlideHeight.PRELOAD_DROP

        robot.update();
        robot.setPoseEstimate(new Pose2d(12, -62, Math.toRadians(90)));

        cameraDelayTimer.reset();

        while (!isStopRequested()) {
            robot.update();

            switch (camera) {
                case WAIT:
                    if (cameraDelayTimer.seconds() > .8) {
                        camera = Camera.SAVE;
                    }
                    break;
                case SAVE:
                    pos = pipeline.getPos();
                    switch (pos) {
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


        }
    }
}
