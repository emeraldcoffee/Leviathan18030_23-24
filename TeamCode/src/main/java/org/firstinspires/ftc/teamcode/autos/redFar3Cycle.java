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
@Autonomous(name = "redFar3Cycle")
public class redFar3Cycle extends LinearOpMode {

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

        TrajectorySequence cycleReturn = driveTrain.trajectorySequenceBuilder(new Pose2d(-60, -11, Math.toRadians(180)))
                .lineTo(new Vector2d(-59.9, -11))
                .splineToConstantHeading(new Vector2d(-34, -6.6), Math.toRadians(0))
                .lineTo(new Vector2d(19, -6.6))
                .addTemporalMarker(2, () -> targetSlidePos = RobotConstants.slideLow)
                .splineToConstantHeading(new Vector2d(54.3, -28.5), Math.toRadians(0))
                .addTemporalMarker(3.1, () -> robot.dropServo.setPosition(RobotConstants.dropPartial))
                .waitSeconds(0.3)
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
                //6.12
                .addDisplacementMarker(() -> targetSlidePos = RobotConstants.slideBottom)
                .lineTo(new Vector2d(-35, -40))
                .splineToConstantHeading(new Vector2d(-31, -34), Math.toRadians(100))
                .addSpatialMarker(new Vector2d(-31, -34), () -> robot.rightServo.setPosition(RobotConstants.rightIn))
                .waitSeconds(.2)
                .lineTo(new Vector2d(-38, -33))
                .splineToSplineHeading(new Pose2d(-41, -20, Math.toRadians(0)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-32, -10.5), Math.toRadians(0))
                .lineTo(new Vector2d(-12, -10.5))
                //.waitSeconds(13)
                .lineTo(new Vector2d(10, -10.5))
                .splineToConstantHeading(new Vector2d(55, -29), Math.toRadians(0))
                .addSpatialMarker(new Vector2d(25, -22), () -> targetSlidePos = RobotConstants.slideAuto)
                .addTemporalMarker(6.12 + 0.61, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
                //add 6.12 + 0.61 + 0.6 TO WAIT
                .addTemporalMarker(6.12 + 0.61 + 0.6, () -> {
                    driveTrain.followTrajectorySequenceAsync(goToStackFromPath);
                })
                .build();

        //changed
        TrajectorySequence center = driveTrain.trajectorySequenceBuilder(new Pose2d(12, -63, Math.toRadians(90)))
                //4.97
                .addDisplacementMarker(() -> {})//targetSlidePos = RobotConstants.slideBottom)
                .lineTo(new Vector2d(-35, -60))
                .splineToSplineHeading(new Pose2d(-36, -17, Math.toRadians(0)), Math.toRadians(100))
                .addSpatialMarker(new Vector2d(-36, -17), () -> {})//robot.rightServo.setPosition(RobotConstants.rightIn))
                .waitSeconds(.2)
                .lineTo(new Vector2d(-36, -16.5))
                .splineToConstantHeading(new Vector2d(-12, -12), Math.toRadians(0))
                //.waitSeconds(13)
                .lineTo(new Vector2d(30, -12))
                .splineToConstantHeading(new Vector2d(55, -29), Math.toRadians(0))
                .addSpatialMarker(new Vector2d(26, -22), () -> targetSlidePos = RobotConstants.slideAuto)
                .addTemporalMarker(4.97 + 0.61, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
                //time to wait end of trajectory + for robot to stop at panel + time for pixel to drop
                .addTemporalMarker(4.97 + 0.61 + 0.6, () -> {
                    driveTrain.followTrajectorySequenceAsync(goToStackFromPath);
                })
                .build();

        //changed
        TrajectorySequence right = driveTrain.trajectorySequenceBuilder(new Pose2d(12, -63, Math.toRadians(90)))
                .addDisplacementMarker(() -> targetSlidePos = RobotConstants.slideBottom)
                .lineTo(new Vector2d(-35, -40))
                .splineToConstantHeading(new Vector2d(-31, -34), Math.toRadians(100))
                .addSpatialMarker(new Vector2d(-31, -34), () -> {})//robot.rightServo.setPosition(RobotConstants.rightIn))
                .waitSeconds(.2)
                .lineTo(new Vector2d(-38, -33))
                .splineToSplineHeading(new Pose2d(-41, -20, Math.toRadians(0)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-32, -10.5), Math.toRadians(0))
                .lineTo(new Vector2d(-12, -10.5))
                //TO ADD A WAIT REMEMBER TO ACCOUNT FOR WAIT SECONDS
                //.waitSeconds(9)
                .lineTo(new Vector2d(25, -10.5))
                .splineToConstantHeading(new Vector2d(55, -29), Math.toRadians(0))
                .addTemporalMarker(4.3, () -> targetSlidePos = RobotConstants.slideAuto)
                .addTemporalMarker(7.2 , () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
                //0.2 is amt of time to remove pixel
                .addTemporalMarker(5.46 + 0.6, () -> {
                    driveTrain.followTrajectorySequenceAsync(goToStackFromPath);
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
                        camera = redFar3Cycle.Camera.SAVE;
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

                    camera = redFar3Cycle.Camera.FINISHED;
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