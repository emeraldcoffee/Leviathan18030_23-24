package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.ColorMask;
import org.firstinspires.ftc.teamcode.robot.HwMap;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.RobotMethods;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Blue Far 2+1")
public class blueFar2plus1 extends LinearOpMode {
    enum Camera {
        WAIT,
        SAVE,
        FINISHED
    }

    fCamRedFarAutoAvoid.Camera camera = fCamRedFarAutoAvoid.Camera.WAIT;

    int targetSlidePos = RobotConstants.slideAuto;

    double intakePos = RobotConstants.stackMax;

    double slideI = 0;

    String pos = "";

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);
        ColorMask pipeline = new ColorMask();
        HwMap robot = new HwMap();
        robot.init(hardwareMap);

        robot.transferMotor.setPower(-.2);
        robot.spikeMarkHoldServo.setPosition(RobotConstants.holdServoUp);

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        robot.webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"));

        robot.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                pipeline.setAlliance("Blue");
                robot.webcam.setPipeline(pipeline);

                robot.webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override

            public void onError(int errorCode) {
                telemetry.addData("Error: ", errorCode);
            }
        });

        TrajectorySequence right = driveTrain.trajectorySequenceBuilder(new Pose2d(-35, 62, Math.toRadians(270)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(310), 10.62))
                .splineToSplineHeading(new Pose2d(-47, 27, Math.toRadians(0)), Math.toRadians(270))
                .addTemporalMarker(.1, () -> targetSlidePos = RobotConstants.slideBottom)
                .lineTo(new Vector2d(-47, 34))
                .addTemporalMarker(2.3, () -> {robot.rightServo.setPosition(RobotConstants.rightIn);})
                .addTemporalMarker(1.2, () -> {
                                    robot.intakeMotor.setPower(1);
                                    robot.transferMotor.setPower(1);
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack);
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack+RobotConstants.rightSpikeOffset);

                })
                .splineToConstantHeading(new Vector2d(-57.5, 34), Math.toRadians(180))
                .addTemporalMarker(3.1, () -> {
                                    robot.spikeMarkHoldServo.setPosition(RobotConstants.holdServoDown);
                })
                .addTemporalMarker(3.2, () -> {
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn+RobotConstants.rightSpikeOffset);
                })
                .addTemporalMarker(3.5, () -> {
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack);
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack+RobotConstants.rightSpikeOffset);
                })
                .addTemporalMarker(3.7, () -> {
                                    robot.spikeMarkHoldServo.setPosition(RobotConstants.holdServoUp);
                })
                .waitSeconds(.5+4)
                .lineTo(new Vector2d(-53, 35))
                .splineToConstantHeading(new Vector2d(-33, 60.5), Math.toRadians(0))
                .lineTo(new Vector2d(25, 60.5))
                .addTemporalMarker(6.1+4, () -> {
                                    targetSlidePos = RobotConstants.slideAuto;
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide);
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide+RobotConstants.rightSpikeOffset);
                                    robot.intakeMotor.setPower(0);
                                    robot.transferMotor.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(46, 25.5), Math.toRadians(0))
                .lineTo(new Vector2d(49, 25.5))
                .addTemporalMarker(8.1+4, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
                .addTemporalMarker(8.7+4, () -> {
                                    targetSlidePos = RobotConstants.slideLow;
                                    robot.dropServo.setPosition(RobotConstants.dropPartial);
                })
                .waitSeconds(.9)
                .addTemporalMarker(9.5+4, () -> targetSlidePos = RobotConstants.slideBottom)
                .lineTo(new Vector2d(45, 25.5))
                .lineTo(new Vector2d(45, 60))
                .lineTo(new Vector2d(55, 60))
                .build();

        TrajectorySequence center = driveTrain.trajectorySequenceBuilder(new Pose2d(-35, 62, Math.toRadians(270)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(310), 10.62))
                .splineToSplineHeading(new Pose2d(-40, 32, Math.toRadians(0)), Math.toRadians(270))
                .addTemporalMarker(.1, () -> targetSlidePos = RobotConstants.slideBottom)
                .lineTo(new Vector2d(-40, 33))
                .addTemporalMarker(1.5, () -> {robot.rightServo.setPosition(RobotConstants.rightIn);})
                .addTemporalMarker(1.4, () -> {
                                    robot.intakeMotor.setPower(1);
                                    robot.transferMotor.setPower(1);
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack);
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack+RobotConstants.rightSpikeOffset);

                })
                .splineToConstantHeading(new Vector2d(-56, 35), Math.toRadians(180))
                .addTemporalMarker(2.9, () -> {
                                    robot.spikeMarkHoldServo.setPosition(RobotConstants.holdServoDown);
                })
                .addTemporalMarker(3.0, () -> {
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn+RobotConstants.rightSpikeOffset);
                })
                .addTemporalMarker(3.3, () -> {
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack);
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack+RobotConstants.rightSpikeOffset);
                })
                .addTemporalMarker(3.5, () -> {
                                    robot.spikeMarkHoldServo.setPosition(RobotConstants.holdServoUp);
                })
                .waitSeconds(.5+4)
                .lineTo(new Vector2d(-53, 35))
                .splineToConstantHeading(new Vector2d(-33, 60), Math.toRadians(0))
                .lineTo(new Vector2d(31, 60))
                .addTemporalMarker(6.1+4, () -> {
                                    targetSlidePos = RobotConstants.slideAuto;
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide);
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide+RobotConstants.rightSpikeOffset);
                                    robot.intakeMotor.setPower(0);
                                    robot.transferMotor.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(47, 32.5), Math.toRadians(0))
                .lineTo(new Vector2d(51, 32.5))
                .addTemporalMarker(7.7+4, () -> {
                                        robot.dropServo.setPosition(RobotConstants.dropOpen);
                })
                .addTemporalMarker(8.3+4, () -> {
                                    robot.dropServo.setPosition(RobotConstants.dropPartial);
                                    targetSlidePos = RobotConstants.slideLow;
                })
                .waitSeconds(.9)
                .addTemporalMarker(9.0+4, () -> targetSlidePos = RobotConstants.slideBottom)
                .lineTo(new Vector2d(45, 32.5))
                .lineTo(new Vector2d(45, 63))
                .lineTo(new Vector2d(55, 63))
                .build();

        TrajectorySequence left = driveTrain.trajectorySequenceBuilder(new Pose2d(-35, 62, Math.toRadians(270)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(310), 10.62))
                .splineToSplineHeading(new Pose2d(-36, 35, Math.toRadians(0)), Math.toRadians(270))
                .addTemporalMarker(.1, () -> targetSlidePos = RobotConstants.slideBottom)
                .lineTo(new Vector2d(-23, 35))
                .addTemporalMarker(2.6, () -> robot.rightServo.setPosition(RobotConstants.rightIn))
                .lineTo(new Vector2d(-28, 35))
                .addTemporalMarker(3.3, () -> {
                                    robot.intakeMotor.setPower(1);
                                    robot.transferMotor.setPower(1);
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack);
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack+RobotConstants.rightSpikeOffset);

                })
                .splineToConstantHeading(new Vector2d(-55.5, 35), Math.toRadians(180))
                .addTemporalMarker(4.1, () -> {
                                    robot.spikeMarkHoldServo.setPosition(RobotConstants.holdServoDown);
                })
                .addTemporalMarker(4.2, () -> {
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn+RobotConstants.rightSpikeOffset);
                })
                .addTemporalMarker(4.5, () -> {
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack);
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack+RobotConstants.rightSpikeOffset);
                })
                .addTemporalMarker(4.7, () -> {
                                    robot.spikeMarkHoldServo.setPosition(RobotConstants.holdServoUp);
                })
                .waitSeconds(.5+4)
                .lineTo(new Vector2d(-53, 35))
                .splineToConstantHeading(new Vector2d(-33, 60), Math.toRadians(0))
                .lineTo(new Vector2d(33, 60))
                .addTemporalMarker(7.2+4, () -> {
                                    targetSlidePos = RobotConstants.slideAuto;
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide);
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide+RobotConstants.rightSpikeOffset);
                                    robot.intakeMotor.setPower(0);
                                    robot.transferMotor.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(52, 43), Math.toRadians(0))
                .addTemporalMarker(8.5+4, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
                .addTemporalMarker(9.0+4, () -> {
                                    targetSlidePos = RobotConstants.slideLow;
                                    robot.dropServo.setPosition(RobotConstants.dropPartial);
                })
                .waitSeconds(.9)
                .addTemporalMarker(10.0+4, () -> targetSlidePos = RobotConstants.slideBottom)
                .lineTo(new Vector2d(45, 46))
                .lineTo(new Vector2d(45, 63))
                .lineTo(new Vector2d(55, 63))
                .build();

        ElapsedTime cameraDelayTimer = new ElapsedTime();

        telemetry.setAutoClear(false);
        Telemetry.Item detectedPos = telemetry.addData("Position", "No detection");
        Telemetry.Item wasPos = telemetry.addData("Was pos", "");

        Telemetry.Item slideData = telemetry.addData("Slide Data:", "Encoder Val:" + robot.liftEncoder.getCurrentPosition() + " Target Val:" + targetSlidePos);


        robot.rightServo.setPosition(RobotConstants.rightOut);
        robot.dropServo.setPosition(RobotConstants.dropClosed);

        waitForStart();
        if (isStopRequested()) return;

        driveTrain.update();
        driveTrain.setPoseEstimate(new Pose2d(-35, 62, Math.toRadians(270)));

        cameraDelayTimer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            driveTrain.update();

            switch (camera) {
                case WAIT:
                    if (cameraDelayTimer.seconds() > 1.5) {
                        camera = fCamRedFarAutoAvoid.Camera.SAVE;
                    }
                    break;
                case SAVE:
                    pos = pipeline.getPos();
                    switch (pos) {
                        case "left":
                            driveTrain.followTrajectorySequenceAsync(left);
                            detectedPos.setValue("Left");
                            break;
                        case "center":
                            driveTrain.followTrajectorySequenceAsync(center);
                            detectedPos.setValue("Center");
                            break;
                        case "right":
                            driveTrain.followTrajectorySequenceAsync(right);
                            detectedPos.setValue("Right");
                            break;
                        default:
                            driveTrain.followTrajectorySequenceAsync(center);
                            detectedPos.setValue("Default center (No detection)");
                            break;
                    }
                    wasPos.setValue(RobotMethods.updateRobotPosition(driveTrain.getPoseEstimate()));

                    camera = fCamRedFarAutoAvoid.Camera.FINISHED;
                    break;
                case FINISHED:
                    break;
            }

            double slideVelo = robot.liftEncoder.getCorrectedVelocity();
            int slideCurPos = robot.liftEncoder.getCurrentPosition();

            double distRemain = targetSlidePos - slideCurPos;

            slideI += distRemain * RobotConstants.slidePIDVals.i;

            double slidePower = (distRemain * RobotConstants.slidePIDVals.p) + slideI + (slideVelo * RobotConstants.slidePIDVals.d);

            robot.slideMotor.setPower(slidePower);

            slideData.setValue("Encoder Val: " + slideCurPos + " Target Val: " + targetSlidePos + " Slide Power: " + (double) Math.round(slidePower * 100) / 100);

        }
    }
}