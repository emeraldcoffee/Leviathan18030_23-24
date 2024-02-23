package org.firstinspires.ftc.teamcode.autos.old;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.ColorMask;
import org.firstinspires.ftc.teamcode.robot.HwMap;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.RobotMethods;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous(name = "BlueClose2Plus2")
public class BlueClose2Plus2 extends LinearOpMode {

    enum Camera {
        WAIT,
        SAVE,
        FINISHED
    }

    Camera camera = Camera.WAIT;

//    enum AutoPath {
//        LEFT,
//        CENTER,
//        RIGHT
//    }
//    AutoPath autoPath = AutoPath.RIGHT;

    int targetSlidePos = RobotConstants.slideAuto;

    double intakePos = RobotConstants.stackMax;

    //double drawbridgeCurrentPos = RobotConstants.stackDrawbridgeUp;
    //double drawbridgeTargetPos = drawbridgeCurrentPos;

    double slideI = 0;

    String pos = "";

    @Override
    public void runOpMode() throws InterruptedException {


        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);
        ColorMask pipeline = new ColorMask();
        HwMap robot = new HwMap();
        robot.init(hardwareMap);

        robot.spikeMarkHoldServo.setPosition(RobotConstants.holdServoUp);
//        robot.leftLiftServo.setPosition(intakePos+RobotConstants.stackLeftOffset);
//        robot.rightLiftServo.setPosition(intakePos);
//        robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
//        robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn + RobotConstants.rightSpikeOffset);

        ElapsedTime autoTimer = new ElapsedTime();

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        robot.webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"));

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

//        robot.leftLiftServo.setPosition(intakePos+RobotConstants.stackLeftOffset);
//        robot.rightLiftServo.setPosition(intakePos);
//        robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
//        robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn + RobotConstants.rightSpikeOffset);

        TrajectorySequence continueOne = driveTrain.trajectorySequenceBuilder(new Pose2d(53, 27, Math.toRadians(0)))
                .build();
        //changed
        TrajectorySequence left = driveTrain.trajectorySequenceBuilder(new Pose2d(12, 63.5, Math.toRadians(270)))
                .lineTo(new Vector2d(12, 61))
                .splineToLinearHeading(new Pose2d(23, 40), Math.toRadians(270))
                .lineTo(new Vector2d(23, 48))
                .addTemporalMarker(2.15, () -> robot.rightServo.setPosition(RobotConstants.rightIn))
                .lineTo(new Vector2d(26, 48))
                .splineToConstantHeading(new Vector2d(47, 44), Math.toRadians(0))
                .lineTo(new Vector2d(53, 44))
                .addTemporalMarker(3.8, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
                .waitSeconds(.2)
                .lineToConstantHeading(new Vector2d(52, 44))
                .addTemporalMarker(4, () -> {
                                    robot.transferMotor.setPower(.3);
                })
                .addTemporalMarker(5, () -> targetSlidePos = RobotConstants.slideBottom)

                .addTemporalMarker(6.4, () -> {
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack + RobotConstants.rightSpikeOffset);
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack);
                                    robot.intakeMotor.setPower(1);
                                    robot.transferMotor.setPower(1);
                                    robot.dropServo.setPosition(RobotConstants.dropClosed);
                })

                .splineToConstantHeading(new Vector2d(9, 12), Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-56.5, 12))

                //8.07, reaches stack
                //take in pixels

                .waitSeconds(0.4)

                .addTemporalMarker(0.1 + 8.07, () -> {
                                    robot.spikeMarkHoldServo.setPosition(RobotConstants.holdServoDown);
                })
                .addTemporalMarker(0.1 + 0.1 + 8.07, () -> {
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn+RobotConstants.rightSpikeOffset);
                })
                //0.2 to take in two pixels
                .addTemporalMarker(0.2 + 0.1 + 0.1 + 8.07, () -> {
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack);
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack+RobotConstants.rightSpikeOffset);
                })

                .waitSeconds(0.1)

                // second

                .waitSeconds(0.5)
                .addTemporalMarker(0.1 + 0.5 + 8.07, () -> {
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn + RobotConstants.rightSpikeOffset);
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                })
                .addTemporalMarker(0.2 + 0.1 + 0.5 + 8.07, () -> {
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack + RobotConstants.rightSpikeOffset);
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack);
                })
                .addTemporalMarker(0.1 + 0.2 + 0.1 + 0.5 + 8.07, () -> {
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn + RobotConstants.rightSpikeOffset);
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                })
                .addTemporalMarker(0.1 + 0.1 + 0.2 + 0.1 + 0.5 + 8.07, () -> {
                                   robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide);
                                   robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide+RobotConstants.rightSpikeOffset);
                })

                .waitSeconds(.5+4)
                .addTemporalMarker(3.93 + 8.07 + 4, () -> {
                                    targetSlidePos = RobotConstants.slideAuto;
                })

                .lineToConstantHeading(new Vector2d(9, 12))
                .addTemporalMarker( 3.43 + 8.07 + 4, () -> {
                                    targetSlidePos = RobotConstants.slideAuto;
                                    robot.intakeMotor.setPower(0);
                                    robot.transferMotor.setPower(0);
                })

                .splineToConstantHeading(new Vector2d(53, 27), Math.toRadians(0))

                //drop 2 pixels
                //13.05 + 4 = 17.05
                .waitSeconds(0.2)

                .addTemporalMarker(0.1 + 13.05 + 4, () -> robot.dropServo.setPosition(RobotConstants.dropPartial))
                .build();

        TrajectorySequence center = driveTrain.trajectorySequenceBuilder(new Pose2d(12, 63.5, Math.toRadians(270)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(310), 10.62))
                .lineTo(new Vector2d(12, 60))
                .splineToSplineHeading(new Pose2d(17, 33), Math.toRadians(270))
                .lineTo(new Vector2d(17,36))
                .addTemporalMarker(1.9, () -> robot.rightServo.setPosition(RobotConstants.rightIn))
                .lineTo(new Vector2d(18, 36))
                .splineToConstantHeading(new Vector2d(45, 37.5), Math.toRadians(0))
                .lineTo(new Vector2d(53, 37.5))
                .addTemporalMarker(3.9, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
                .waitSeconds(.3)
                .lineTo(new Vector2d(51, 37))
                .addTemporalMarker(4.2, () -> {
                                    robot.transferMotor.setPower(.3);
                })
                .addTemporalMarker(5, () -> targetSlidePos = RobotConstants.slideBottom)
                .addTemporalMarker(6, () -> {
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack + RobotConstants.rightSpikeOffset);
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack);
                                    robot.intakeMotor.setPower(1);
                                    robot.transferMotor.setPower(1);
                                    robot.dropServo.setPosition(RobotConstants.dropClosed);
                })
                .splineToConstantHeading(new Vector2d(9, 12), Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-56.5, 12))


                //7.8, reaches stack
                //take in pixels

                .waitSeconds(0.4)

                .addTemporalMarker(0.1 + 7.8, () -> {
                                    robot.spikeMarkHoldServo.setPosition(RobotConstants.holdServoDown);
                })
                .addTemporalMarker(0.1 + 0.1 + 7.8, () -> {
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn+RobotConstants.rightSpikeOffset);
                })
                //0.2 to take in two pixels
                .addTemporalMarker(0.2 + 0.1 + 0.1 + 7.8, () -> {
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack);
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack+RobotConstants.rightSpikeOffset);
                })

                .waitSeconds(0.1)

                // second

                .waitSeconds(0.5)
                .addTemporalMarker(0.1 + 0.5 + 7.8, () -> {
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn + RobotConstants.rightSpikeOffset);
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                })
                .addTemporalMarker(0.2 + 0.1 + 0.5 + 7.8, () -> {
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack + RobotConstants.rightSpikeOffset);
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack);
                })
                .addTemporalMarker(0.1 + 0.2 + 0.1 + 0.5 + 7.8, () -> {
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn + RobotConstants.rightSpikeOffset);
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                })
                .addTemporalMarker(0.1 + 0.1 + 0.2 + 0.1 + 0.5 + 7.8, () -> {
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide);
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide+RobotConstants.rightSpikeOffset);
                })

                .waitSeconds(.5+4)


                .lineToConstantHeading(new Vector2d(9, 12))

                .addTemporalMarker(3.5 + 7.8 + 4, () -> {
                                    targetSlidePos = RobotConstants.slideAuto;
                                    robot.intakeMotor.setPower(0);
                                    robot.transferMotor.setPower(0);
                })

                .splineToConstantHeading(new Vector2d(53, 27), Math.toRadians(0))

                //time up to here 16.78
                //drop 2 pixels
                //4.98 + 4 + 7.8 = 16.78
                .waitSeconds(0.2)

                .addTemporalMarker(0.15 + 4.98 + 4 + 7.8, () -> robot.dropServo.setPosition(RobotConstants.dropPartial))

                .build();


        TrajectorySequence right = driveTrain.trajectorySequenceBuilder(new Pose2d(12, 63.5, Math.toRadians(270)))
                .lineTo(new Vector2d(12, 45))
                .splineToConstantHeading(new Vector2d(8.5, 36), Math.toRadians(180))
                .lineTo(new Vector2d(7, 36))
                .addTemporalMarker(1.6, () -> robot.rightServo.setPosition(RobotConstants.rightIn))
                .lineTo(new Vector2d(20, 36))
                .splineToSplineHeading(new Pose2d(45, 28.5, Math.toRadians(0)), Math.toRadians(0))
                .lineTo(new Vector2d(53, 28.5))
                .addTemporalMarker(3.7, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
                .waitSeconds(.3)
                .lineTo(new Vector2d(52, 28.5))

                .addTemporalMarker(4.2, () -> {
                                    robot.transferMotor.setPower(.3);
                })
                .addTemporalMarker(5, () -> targetSlidePos = RobotConstants.slideBottom)
                .addTemporalMarker(6, () -> {
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack + RobotConstants.rightSpikeOffset);
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack);
                                    robot.intakeMotor.setPower(1);
                                    robot.transferMotor.setPower(1);
                                    robot.dropServo.setPosition(RobotConstants.dropClosed);
                })
                .splineToConstantHeading(new Vector2d(9, 12), Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-56.5, 12))


                //7.61, reaches stack
                //take in pixels

                .waitSeconds(0.4)

                .addTemporalMarker(0.1 + 7.61, () -> {
                                    robot.spikeMarkHoldServo.setPosition(RobotConstants.holdServoDown);
                })
                .addTemporalMarker(0.1 + 0.1 + 7.61, () -> {
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn+RobotConstants.rightSpikeOffset);
                })
                //0.2 to take in two pixels
                .addTemporalMarker(0.2 + 0.1 + 0.1 + 7.61, () -> {
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack);
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack+RobotConstants.rightSpikeOffset);
                })

                .waitSeconds(0.1)

                // second

                .waitSeconds(0.5)
                .addTemporalMarker(0.1 + 0.5 + 7.61, () -> {
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn + RobotConstants.rightSpikeOffset);
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                })
                .addTemporalMarker(0.2 + 0.1 + 0.5 + 7.61, () -> {
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack + RobotConstants.rightSpikeOffset);
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack);
                })
                .addTemporalMarker(0.1 + 0.2 + 0.1 + 0.5 + 7.61, () -> {
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn + RobotConstants.rightSpikeOffset);
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                })
                .addTemporalMarker(0.1 + 0.1 + 0.2 + 0.1 + 0.5 + 7.61, () -> {
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide);
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide+RobotConstants.rightSpikeOffset);
                })

                .waitSeconds(.5+4)


                .lineToConstantHeading(new Vector2d(9, 12))

                .addTemporalMarker(3.5 + 7.61 + 4, () -> {
                                    targetSlidePos = RobotConstants.slideAuto;
                                    robot.intakeMotor.setPower(0);
                                    robot.transferMotor.setPower(0);
                })

                .splineToConstantHeading(new Vector2d(52, 28.5), Math.toRadians(0))

                //time up to here 16.74
                //drop 2 pixels

                .waitSeconds(0.2)

                .addTemporalMarker(0.15 + 4.98 + 4 + 7.61, () -> robot.dropServo.setPosition(RobotConstants.dropPartial))
                .build();



        ElapsedTime cameraDelayTimer = new ElapsedTime();

        telemetry.setAutoClear(false);
        Telemetry.Item detectedPos = telemetry.addData("Position", "No detection");
        Telemetry.Item wasPos = telemetry.addData("Was pos", "");
        Telemetry.Item slideData = telemetry.addData("Slide Data:", "Encoder Val:" + robot.liftEncoder.getCurrentPosition() + " Target Val:" + targetSlidePos);

        robot.rightServo.setPosition(RobotConstants.rightOut);
        robot.dropServo.setPosition(RobotConstants.dropClosed);
        robot.transferMotor.setPower(-.2);

        waitForStart();
        if (isStopRequested()) return;

        robot.climbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveTrain.update();
        driveTrain.setPoseEstimate(new Pose2d(12, 63.5, Math.toRadians(270)));

        cameraDelayTimer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            driveTrain.update();

            switch (camera) {
                case WAIT:
                    if (cameraDelayTimer.seconds() > 1.5) {
                        camera = Camera.SAVE;
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

                    camera = Camera.FINISHED;
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

            slideData.setValue( "Encoder Val: " + slideCurPos + " Target Val: " + targetSlidePos + " Slide Power: " + (double)Math.round(slidePower*100)/100);

            //Limits max speed servos move
//            if (drawbridgeTargetPos<drawbridgeCurrentPos) {
//                drawbridgeCurrentPos+= Range.clip((drawbridgeTargetPos-drawbridgeCurrentPos), -.015, -.0);
//                robot.rightDrawbridgeServo.setPosition(drawbridgeCurrentPos+RobotConstants.drawbridgeRightOffset);
//                robot.leftDrawbridgeServo.setPosition(drawbridgeCurrentPos);
//            } else if (drawbridgeTargetPos>drawbridgeCurrentPos) {
//                drawbridgeCurrentPos+=Range.clip((drawbridgeTargetPos-drawbridgeCurrentPos), 0, .015);
//                robot.rightDrawbridgeServo.setPosition(drawbridgeCurrentPos+RobotConstants.drawbridgeRightOffset);
//                robot.leftDrawbridgeServo.setPosition(drawbridgeCurrentPos);
        }
    }

}
