package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
@Autonomous(name = "redClose3Cycle")
public class redClose3Cycle extends LinearOpMode {

    enum Camera {
        WAIT,
        SAVE,
        FINISHED
    }

    Camera camera = Camera.WAIT;

    //enum AutoPath {
    //      LEFT,
    //     CENTER,
    //    RIGHT

    // AutoPath autoPath = AutoPath.RIGHT;

    int targetSlidePos = RobotConstants.slideAuto;

    double intakePos = RobotConstants.stackMax;

    double drawbridgeCurrentPos = RobotConstants.stackDrawbridgeUp;
    double drawbridgeTargetPos = drawbridgeCurrentPos;

    double slideI = 0;

    String pos = "";

    int counter = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);
        ColorMask pipeline = new ColorMask();
        HwMap robot = new HwMap();
        robot.init(hardwareMap);

//        robot.leftLiftServo.setPosition(intakePos+RobotConstants.stackLeftOffset);
//        robot.rightLiftServo.setPosition(intakePos);
        robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
        robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn + RobotConstants.rightSpikeOffset);

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

//        robot.leftLiftServo.setPosition(intakePos+RobotConstants.stackLeftOffset);
//        robot.rightLiftServo.setPosition(intakePos);
        robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
        robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn + RobotConstants.rightSpikeOffset);

        TrajectorySequence park = driveTrain.trajectorySequenceBuilder(new Pose2d(-60, -11, Math.toRadians(180)))
                .lineTo(new Vector2d(37, -12))
                .waitSeconds(0.2)
                .lineTo(new Vector2d(54, -12))
                .build();


        TrajectorySequence cycleReturn = driveTrain.trajectorySequenceBuilder(new Pose2d(-60, -11, Math.toRadians(180)))
                //go to stack
                .lineTo(new Vector2d(-34, -6.6))
                .splineToConstantHeading(new Vector2d(-60, -11), Math.toRadians(180))
                .addTemporalMarker(3, () -> {
                    robot.transferMotor.setPower(.3);
                })
                .addTemporalMarker(4, () -> targetSlidePos = RobotConstants.slideBottom)

                .addTemporalMarker(5.4, () -> {
                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide + RobotConstants.rightSpikeOffset);
                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide);
                    robot.intakeMotor.setPower(1);
                    robot.transferMotor.setPower(1);
                    robot.dropServo.setPosition(RobotConstants.dropClosed);
                })

                .addTemporalMarker(6.5, () -> {
                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn + RobotConstants.rightSpikeOffset);
                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                })
                .addTemporalMarker(7, () -> {
                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide + RobotConstants.rightSpikeOffset);
                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide);
                })
                .addTemporalMarker(7.5, () -> {
                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn + RobotConstants.rightSpikeOffset);
                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                })
                .waitSeconds(0.2)

                //cycle return
                .lineTo(new Vector2d(-59.9, -11))
                .splineToConstantHeading(new Vector2d(-34, -6.6), Math.toRadians(0))
                .lineTo(new Vector2d(19, -6.6))
                .addTemporalMarker(2, () -> targetSlidePos = RobotConstants.slideLow)
                .splineToConstantHeading(new Vector2d(54.3, -28.5), Math.toRadians(0))
                .addTemporalMarker(3.1, () -> robot.dropServo.setPosition(RobotConstants.dropPartial))
                .waitSeconds(0.3)

                //second
                .lineToConstantHeading(new Vector2d(54.2, -28.5))
                .splineToConstantHeading(new Vector2d(19, -6.6), Math.toRadians(180))

                .lineTo(new Vector2d(-34, -6.6))
                .splineToConstantHeading(new Vector2d(-60, -11), Math.toRadians(180))
                .addTemporalMarker(3, () -> {
                    robot.transferMotor.setPower(.3);
                })
                .addTemporalMarker(4, () -> targetSlidePos = RobotConstants.slideBottom)

                .addTemporalMarker(5.4, () -> {
                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide + RobotConstants.rightSpikeOffset);
                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide);
                    robot.intakeMotor.setPower(1);
                    robot.transferMotor.setPower(1);
                    robot.dropServo.setPosition(RobotConstants.dropClosed);
                })

                .addTemporalMarker(6.5, () -> {
                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn + RobotConstants.rightSpikeOffset);
                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                })
                .addTemporalMarker(7, () -> {
                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide + RobotConstants.rightSpikeOffset);
                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide);
                })
                .addTemporalMarker(7.5, () -> {
                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn + RobotConstants.rightSpikeOffset);
                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                })

                .waitSeconds(0.2)

                .lineTo(new Vector2d(-59.9, -11))
                .splineToConstantHeading(new Vector2d(-34, -6.6), Math.toRadians(0))
                .lineTo(new Vector2d(19, -6.6))
                .addTemporalMarker(2, () -> targetSlidePos = RobotConstants.slideLow)
                .splineToConstantHeading(new Vector2d(54.3, -28.5), Math.toRadians(0))
                .addTemporalMarker(3.1, () -> robot.dropServo.setPosition(RobotConstants.dropPartial))
                .waitSeconds(0.3)
                /*.addTemporalMarker(7.7, () -> {
                    driveTrain.followTrajectorySequenceAsync(cycleReturn);
                })*/

                //third
                .lineToConstantHeading(new Vector2d(54.2, -28.5))
                .splineToConstantHeading(new Vector2d(19, -6.6), Math.toRadians(180))

                .lineTo(new Vector2d(-34, -6.6))
                .splineToConstantHeading(new Vector2d(-60, -11), Math.toRadians(180))
                .addTemporalMarker(3, () -> {
                    robot.transferMotor.setPower(.3);
                })
                .addTemporalMarker(4, () -> targetSlidePos = RobotConstants.slideBottom)

                .addTemporalMarker(5.4, () -> {
                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide + RobotConstants.rightSpikeOffset);
                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide);
                    robot.intakeMotor.setPower(1);
                    robot.transferMotor.setPower(1);
                    robot.dropServo.setPosition(RobotConstants.dropClosed);
                })

                .addTemporalMarker(6.5, () -> {
                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn + RobotConstants.rightSpikeOffset);
                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                })
                .addTemporalMarker(7, () -> {
                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide + RobotConstants.rightSpikeOffset);
                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide);
                })
                .addTemporalMarker(7.5, () -> {
                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn + RobotConstants.rightSpikeOffset);
                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                })
                .waitSeconds(0.2)

                .lineTo(new Vector2d(-59.9, -11))
                .splineToConstantHeading(new Vector2d(-34, -6.6), Math.toRadians(0))
                .lineTo(new Vector2d(19, -6.6))
                .addTemporalMarker(2, () -> targetSlidePos = RobotConstants.slideLow)
                .splineToConstantHeading(new Vector2d(54.3, -28.5), Math.toRadians(0))
                .addTemporalMarker(3.1, () -> robot.dropServo.setPosition(RobotConstants.dropPartial))
                .waitSeconds(0.3)

                //fourth
                .lineToConstantHeading(new Vector2d(54.2, -28.5))
                .splineToConstantHeading(new Vector2d(19, -6.6), Math.toRadians(180))

                .lineTo(new Vector2d(-34, -6.6))
                .splineToConstantHeading(new Vector2d(-60, -11), Math.toRadians(180))
                .addTemporalMarker(3, () -> {
                    robot.transferMotor.setPower(.3);
                })
                .addTemporalMarker(4, () -> targetSlidePos = RobotConstants.slideBottom)

                .addTemporalMarker(5.4, () -> {
                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide + RobotConstants.rightSpikeOffset);
                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide);
                    robot.intakeMotor.setPower(1);
                    robot.transferMotor.setPower(1);
                    robot.dropServo.setPosition(RobotConstants.dropClosed);
                })

                .addTemporalMarker(6.5, () -> {
                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn + RobotConstants.rightSpikeOffset);
                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                })
                .addTemporalMarker(7, () -> {
                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide + RobotConstants.rightSpikeOffset);
                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide);
                })
                .addTemporalMarker(7.5, () -> {
                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn + RobotConstants.rightSpikeOffset);
                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                })
                .waitSeconds(0.2)

                .lineTo(new Vector2d(-59.9, -11))
                .splineToConstantHeading(new Vector2d(-34, -6.6), Math.toRadians(0))
                .lineTo(new Vector2d(19, -6.6))
                .addTemporalMarker(2, () -> targetSlidePos = RobotConstants.slideLow)
                .splineToConstantHeading(new Vector2d(54.3, -28.5), Math.toRadians(0))
                .addTemporalMarker(3.1, () -> robot.dropServo.setPosition(RobotConstants.dropPartial))
                .waitSeconds(0.3)

                //park
                .lineTo(new Vector2d(37, -12))
                .waitSeconds(0.2)
                .lineTo(new Vector2d(54, -12))

                .build();

        TrajectorySequence goToStackFromPath = driveTrain.trajectorySequenceBuilder(new Pose2d(19, -6.6, Math.toRadians(0)))
                .lineTo(new Vector2d(-34, -6.6))
                .splineToConstantHeading(new Vector2d(-60, -11), Math.toRadians(180))
                .addTemporalMarker(3, () -> {
                    robot.transferMotor.setPower(.3);
                })
                .addTemporalMarker(4, () -> targetSlidePos = RobotConstants.slideBottom)

                .addTemporalMarker(5.4, () -> {
                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide + RobotConstants.rightSpikeOffset);
                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide);
                    robot.intakeMotor.setPower(1);
                    robot.transferMotor.setPower(1);
                    robot.dropServo.setPosition(RobotConstants.dropClosed);
                })

                .addTemporalMarker(6.5, () -> {
                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn + RobotConstants.rightSpikeOffset);
                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                })
                .addTemporalMarker(7, () -> {
                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide + RobotConstants.rightSpikeOffset);
                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide);
                })
                .addTemporalMarker(7.5, () -> {
                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn + RobotConstants.rightSpikeOffset);
                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                })
                .addTemporalMarker(7.7, () -> {
                    driveTrain.followTrajectorySequenceAsync(cycleReturn);
                })
                .build();

        //once you grab something from the stack and want to do it again, follow in the format of continueCycle, goToStackFromPath, and then return cycle
        TrajectorySequence continueCycle = driveTrain.trajectorySequenceBuilder(new Pose2d(54.3, -28.5, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(54.2, -28.5))
                .splineToConstantHeading(new Vector2d(19, -6.6), Math.toRadians(180))

                //call goToStack from path

                .build();


        TrajectorySequence left = driveTrain.trajectorySequenceBuilder(new Pose2d(12, -63, Math.toRadians(90)))
                .lineTo(new Vector2d(12, -45))
                .splineToConstantHeading(new Vector2d(8.5, -36), Math.toRadians(180))
                .lineTo(new Vector2d(5, -36))
                .addTemporalMarker(1.6, () -> robot.leftServo.setPosition(RobotConstants.leftIn))
                .lineTo(new Vector2d(20, -36))
                .splineToSplineHeading(new Pose2d(45, -28.5, Math.toRadians(0)), Math.toRadians(0))
                .lineTo(new Vector2d(54.2, -28.5))
                .lineTo(new Vector2d(54.3, -28.5))
                .addTemporalMarker(4, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
                .waitSeconds(.2)
                .lineToConstantHeading(new Vector2d(54.2, -28.5))
                .splineToConstantHeading(new Vector2d(19, -6.6), Math.toRadians(180))
                .addTemporalMarker(5.62, () -> {
                    driveTrain.followTrajectorySequenceAsync(cycleReturn);
                })
                .build();

        TrajectorySequence center = driveTrain.trajectorySequenceBuilder(new Pose2d(12, -63, Math.toRadians(90)))
                .lineTo(new Vector2d(12, -60))
                .splineToSplineHeading(new Pose2d(17, -30), Math.toRadians(90))
                .lineTo(new Vector2d(17, -36))
                .addTemporalMarker(2.0, () -> robot.leftServo.setPosition(RobotConstants.leftIn))
                .lineTo(new Vector2d(18, -36))
                .splineToConstantHeading(new Vector2d(45, -37.5), Math.toRadians(0))
                .lineTo(new Vector2d(54.2, -37.5))
                .lineTo(new Vector2d(54.3, -37.5))
                .addTemporalMarker(3.9, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
                .waitSeconds(.2)
                .lineToConstantHeading(new Vector2d(54.2, -37.5))
                .splineToConstantHeading(new Vector2d(19, -6.6), Math.toRadians(180))
                .addTemporalMarker(5.99, () -> {
                    driveTrain.followTrajectorySequenceAsync(cycleReturn);
                })
                .build();


        TrajectorySequence right = driveTrain.trajectorySequenceBuilder(new Pose2d(12, -63, Math.toRadians(90)))
                .lineTo(new Vector2d(12, -61))
                .splineToLinearHeading(new Pose2d(23, -40), Math.toRadians(90))
                .lineTo(new Vector2d(23, -46))
                .addTemporalMarker(2.0, () -> robot.leftServo.setPosition(RobotConstants.leftIn))
                .lineTo(new Vector2d(26, -45))
                .splineToConstantHeading(new Vector2d(47, -45), Math.toRadians(0))
                .lineTo(new Vector2d(54.2, -45))
                .lineTo(new Vector2d(54.3, -45))
                .addTemporalMarker(3.8, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
                .waitSeconds(.2)
                .lineToConstantHeading(new Vector2d(54.2, -45))
                .splineToConstantHeading(new Vector2d(19, -6.6), Math.toRadians(180))
                .addTemporalMarker(5.98, () -> {
                    driveTrain.followTrajectorySequenceAsync(cycleReturn);
                })
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

        robot.climbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveTrain.update();
        driveTrain.setPoseEstimate(new Pose2d(12, 63.5, Math.toRadians(270)));

        cameraDelayTimer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            driveTrain.update();

            switch (camera) {
                case WAIT:
                    if (cameraDelayTimer.seconds() > 1.5) {
                        camera = redClose3Cycle.Camera.SAVE;
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

                    camera = redClose3Cycle.Camera.FINISHED;
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

            //Limits max speed servos move
           /* if (drawbridgeTargetPos<drawbridgeCurrentPos) {
              drawbridgeCurrentPos+= Range.clip((drawbridgeTargetPos-drawbridgeCurrentPos), -.015, -.0);
               robot.rightDrawbridgeServo.setPosition(drawbridgeCurrentPos+RobotConstants.drawbridgeRightOffset);
               robot.leftDrawbridgeServo.setPosition(drawbridgeCurrentPos);
           } else if (drawbridgeTargetPos>drawbridgeCurrentPos) {
                drawbridgeCurrentPos+=Range.clip((drawbridgeTargetPos-drawbridgeCurrentPos), 0, .015);
               robot.rightDrawbridgeServo.setPosition(drawbridgeCurrentPos+RobotConstants.drawbridgeRightOffset);
               robot.leftDrawbridgeServo.setPosition(drawbridgeCurrentPos);

            */
        }
    }
}