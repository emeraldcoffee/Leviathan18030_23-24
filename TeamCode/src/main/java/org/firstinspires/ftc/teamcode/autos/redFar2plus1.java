package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.firstinspires.ftc.teamcode.robot.RobotMethods;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Red Far 2+1")
public class redFar2plus1 extends LinearOpMode {
    enum Camera {
        WAIT,
        SAVE,
        FINISHED
    }

    fCamRedFarAutoAvoid.Camera camera = fCamRedFarAutoAvoid.Camera.WAIT;

    String pos = "";

    @Override
    public void runOpMode() throws InterruptedException {

        RobotConfig robot = new RobotConfig(hardwareMap);
//        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);
        ColorMask pipeline = new ColorMask();
//        HwMap robot = new HwMap();
//        robot.init(hardwareMap);

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

        TrajectorySequence right = robot.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(90)))
                .splineToSplineHeading(new Pose2d(-36, -35, Math.toRadians(0)), Math.toRadians(90))
                .addTemporalMarker(.1, () -> {
                                robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                })
                .lineTo(new Vector2d(-23, -35))
                .addTemporalMarker(2.6, () -> {
                                robot.leftPixelServo.setPosition(RobotConstants.leftIn);
                })
                .lineTo(new Vector2d(-28, -35))
                .addTemporalMarker(3.3, () -> {
                                robot.intakeMotor.setPower(1);
                                robot.transferMotor.setPower(1);
                                robot.stackArm(RobotConfig.StackArm.FAR_OUT);
                })
                .splineToConstantHeading(new Vector2d(-55.5, -35), Math.toRadians(180))
                .addTemporalMarker(4.3, () -> {
                                robot.stackHold(true);
                })
                .addTemporalMarker(4.4, () -> {
                                robot.stackArm(RobotConfig.StackArm.IN);
                })
                .addTemporalMarker(4.7, () -> {
                                robot.stackArm(RobotConfig.StackArm.FAR_OUT);
                })
                .addTemporalMarker(4.9, () -> {
                                robot.stackHold(false);
                })
                .waitSeconds(.5)
                .forward(.5)
                .splineToConstantHeading(new Vector2d(-33, -55), Math.toRadians(0))
                .addTemporalMarker(7.2, () -> {
                                robot.setTargetSlidePos(RobotConfig.SlideHeight.PRELOAD_DROP);
                                robot.stackArm(RobotConfig.StackArm.GUIDE);
                                robot.intakeMotor.setPower(0);
                                robot.transferMotor.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(50, -45), Math.toRadians(0))
                .addTemporalMarker(8.8, () -> {
                                robot.dropper(RobotConfig.Dropper.OPEN);
                })
                .addTemporalMarker(9.4, () -> {
                                robot.setTargetSlidePos(RobotConfig.SlideHeight.LOW);
                                robot.dropper(RobotConfig.Dropper.PARTIAL);
                })
                .waitSeconds(.9)
                //Cycle 1
                .back(.1)
                .splineToConstantHeading(new Vector2d(33, -55), Math.toRadians(180))
                .addTemporalMarker(11.1, () -> {
                                robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                })
                .lineTo(new Vector2d(-33, -55))
                .splineToConstantHeading(new Vector2d(-55.5, -35), Math.toRadians(180))
                .addTemporalMarker(13.0, () -> {
                                robot.intakeMotor.setPower(1);
                                robot.transferMotor.setPower(1);
                                robot.dropper(RobotConfig.Dropper.CLOSED);
                })
                .waitSeconds(1)
                .addTemporalMarker(13.4, () -> {
                                robot.stackHold(true);
                })
                .addTemporalMarker(13.5, () -> {
                                robot.stackArm(RobotConfig.StackArm.IN);
                })
                .addTemporalMarker(13.8, () -> {
                                robot.stackArm(RobotConfig.StackArm.OUT);
                })
                .addTemporalMarker(14.0, () -> {
                                robot.stackArm(RobotConfig.StackArm.IN);
                })
                .addTemporalMarker(14.3, () -> {
                                robot.stackArm(RobotConfig.StackArm.FAR_OUT);
                })
                .addTemporalMarker(14.4, () -> {
                                robot.stackHold(false);
                })
                .forward(.1)
                .splineToConstantHeading(new Vector2d(-33, -55), Math.toRadians(0))
                .lineTo(new Vector2d(33, -55))
                .splineToConstantHeading(new Vector2d(50, -45), Math.toRadians(0))
                .addTemporalMarker(17.1, () -> {
                                robot.setTargetSlidePos(RobotConfig.SlideHeight.LOW);
                                robot.transferMotor.setPower(0);
                                robot.intakeMotor.setPower(0);
                })
                .waitSeconds(.5)
                .addTemporalMarker(18.2, () -> {
                                robot.dropper(RobotConfig.Dropper.PARTIAL);
                })
                .waitSeconds(10)
                .build();

        TrajectorySequence center = robot.trajectorySequenceBuilder(new Pose2d(-35, -63, Math.toRadians(90)))
//                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(310), 10.62))
//                .splineToSplineHeading(new Pose2d(-40, -32, Math.toRadians(0)), Math.toRadians(90))
//                .addTemporalMarker(.1, () -> targetSlidePos = RobotConstants.slideBottom)
                .lineTo(new Vector2d(-40, -33))
//                .addTemporalMarker(1.7, () -> {robot.leftPixelServo.setPosition(RobotConstants.leftIn);})
//                .addTemporalMarker(1.4, () -> {
//                                    robot.intakeMotor.setPower(1);
//                                    robot.transferMotor.setPower(1);
//                                    robot.
//                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack);
//                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack+RobotConstants.rightSpikeOffset);
//
//                })
//                .splineToConstantHeading(new Vector2d(-56, -35), Math.toRadians(180))
//                .addTemporalMarker(2.9, () -> {
//                                    robot.spikeMarkHoldServo.setPosition(RobotConstants.holdServoDown);
//                })
//                .addTemporalMarker(3.0, () -> {
//                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
//                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn+RobotConstants.rightSpikeOffset);
//                })
//                .addTemporalMarker(3.3, () -> {
//                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack);
//                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack+RobotConstants.rightSpikeOffset);
//                })
//                .addTemporalMarker(3.5, () -> {
//                                    robot.spikeMarkHoldServo.setPosition(RobotConstants.holdServoUp);
//                })
//                .waitSeconds(.5+4)
//                .lineTo(new Vector2d(-55, -35))
//                .splineToConstantHeading(new Vector2d(-28, -60), Math.toRadians(0))
//                .lineTo(new Vector2d(33, -60))
//                .addTemporalMarker(6.3+4, () -> {
//                                    targetSlidePos = RobotConstants.slideAuto;
//                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide);
//                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide+RobotConstants.rightSpikeOffset);
//                                    robot.intakeMotor.setPower(0);
//                                    robot.transferMotor.setPower(0);
//                })
//                .splineToConstantHeading(new Vector2d(46, -39), Math.toRadians(0))
//                .lineTo(new Vector2d(52, -39))
//                .addTemporalMarker(7.6+4, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
//                .addTemporalMarker(8.1+4, () -> {
//                                    targetSlidePos = RobotConstants.slideLow;
//                                    robot.dropServo.setPosition(RobotConstants.dropPartial);
//                })
//                .waitSeconds(.9)
//                .addTemporalMarker(9.0+4, () -> targetSlidePos = RobotConstants.slideBottom)
//                .lineTo(new Vector2d(45, -39))
//                .lineTo(new Vector2d(45, -64))
                .lineTo(new Vector2d(55, -64))
                .build();

        TrajectorySequence left = robot.trajectorySequenceBuilder(new Pose2d(-35, -63, Math.toRadians(90)))
//                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(310), 10.62))
//                .splineToSplineHeading(new Pose2d(-45, -28, Math.toRadians(0)), Math.toRadians(90))
//                .addTemporalMarker(.1, () -> targetSlidePos = RobotConstants.slideBottom)
                .lineTo(new Vector2d(-45, -34))
//                .addTemporalMarker(2.3, () -> {robot.leftServo.setPosition(RobotConstants.leftIn);})
//                .addTemporalMarker(1.2, () -> {
//                                    robot.intakeMotor.setPower(1);
//                                    robot.transferMotor.setPower(1);
//                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack);
//                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack+RobotConstants.rightSpikeOffset);
//
//                })
//                .splineToConstantHeading(new Vector2d(-56.5, -35), Math.toRadians(180))
//                .addTemporalMarker(3.1, () -> {
//                                    robot.spikeMarkHoldServo.setPosition(RobotConstants.holdServoDown);
//                })
//                .addTemporalMarker(3.2, () -> {
//                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
//                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn+RobotConstants.rightSpikeOffset);
//                })
//                .addTemporalMarker(3.5, () -> {
//                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack);
//                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack+RobotConstants.rightSpikeOffset);
//                })
//                .addTemporalMarker(3.7, () -> {
//                                    robot.spikeMarkHoldServo.setPosition(RobotConstants.holdServoUp);
//                })
//                .waitSeconds(.5+4)
//                .lineTo(new Vector2d(-53, -35))
//                .splineToConstantHeading(new Vector2d(-33, -60), Math.toRadians(0))
//                .lineTo(new Vector2d(33, -60))
//                .addTemporalMarker(6.1+4, () -> {
//                                    targetSlidePos = RobotConstants.slideAuto;
//                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide);
//                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide+RobotConstants.rightSpikeOffset);
//                                    robot.intakeMotor.setPower(0);
//                                    robot.transferMotor.setPower(0);
//                })
//                .splineToConstantHeading(new Vector2d(46, -32.1), Math.toRadians(0))
//                .lineTo(new Vector2d(52, -32.1))
//                .addTemporalMarker(7.9+4, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
//                .addTemporalMarker(8.4+4, () -> {
//                                    targetSlidePos = RobotConstants.slideLow;
//                                    robot.dropServo.setPosition(RobotConstants.dropPartial);
//                })
//                .waitSeconds(.9)
//                .addTemporalMarker(9.4+4, () -> targetSlidePos = RobotConstants.slideBottom)
//                .lineTo(new Vector2d(45, -32.1))
//                .lineTo(new Vector2d(45, -64))
                .lineTo(new Vector2d(55, -64))
                .build();

        ElapsedTime cameraDelayTimer = new ElapsedTime();

        telemetry.setAutoClear(false);
        Telemetry.Item detectedPos = telemetry.addData("Position", "No detection");
        Telemetry.Item wasPos = telemetry.addData("Was pos", "");

//        Telemetry.Item slideData = telemetry.addData("Slide Data:", "Encoder Val:" + robot.liftEncoder.getCurrentPosition() + " Target Val:" + targetSlidePos);


        robot.leftPixelServo.setPosition(RobotConstants.leftOut);
        robot.dropServo.setPosition(RobotConstants.dropClosed);


        waitForStart();
        if (isStopRequested()) return;

        robot.setTargetSlidePos(RobotConfig.SlideHeight.PRELOAD_DROP);

        robot.update();
        robot.setPoseEstimate(new Pose2d(-35, -62, Math.toRadians(90)));

        cameraDelayTimer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            robot.update();

            switch (camera) {
                case WAIT:
                    if (cameraDelayTimer.seconds() > .8) {
                        camera = fCamRedFarAutoAvoid.Camera.SAVE;
                    }
                    break;
                case SAVE:
                    pos = pipeline.getPos();
                    switch ("right") {
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
                    wasPos.setValue(RobotMethods.updateRobotPosition(robot.getPoseEstimate()));

                    camera = fCamRedFarAutoAvoid.Camera.FINISHED;
                    break;
                case FINISHED:
                    break;
            }


        }
    }
}
