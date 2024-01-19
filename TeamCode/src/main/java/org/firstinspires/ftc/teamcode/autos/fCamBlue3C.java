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

@Autonomous(name = "fCamBlue3C")
public class fCamBlue3C extends LinearOpMode {

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

        robot.leftLiftServo.setPosition(intakePos+RobotConstants.stackLeftOffset);
        robot.rightLiftServo.setPosition(intakePos);

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

        robot.leftLiftServo.setPosition(intakePos+RobotConstants.stackLeftOffset);
        robot.rightLiftServo.setPosition(intakePos);

        TrajectorySequence left = driveTrain.trajectorySequenceBuilder(new Pose2d(12, 63, Math.toRadians(270)))
                .lineTo(new Vector2d(12, 61))
                .splineToLinearHeading(new Pose2d(23, 40), Math.toRadians(270))
                .lineTo(new Vector2d(23, 48))
                .addTemporalMarker(2.15, () -> robot.rightServo.setPosition(RobotConstants.rightIn))
                .lineTo(new Vector2d(26, 48))
                .splineToConstantHeading(new Vector2d(47, 44), Math.toRadians(0))
                .lineTo(new Vector2d(53, 44))
                .addTemporalMarker(3.8, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
                .waitSeconds(.2)
                .lineTo(new Vector2d(40, 44))
                .addDisplacementMarker(() -> {targetSlidePos = RobotConstants.slideBottom; robot.dropServo.setPosition(RobotConstants.dropClosed);})
                .lineTo(new Vector2d(40, 60))
                .lineTo(new Vector2d(45, 60))
                .build();

        TrajectorySequence center = driveTrain.trajectorySequenceBuilder(new Pose2d(12, 63, Math.toRadians(270)))
                .lineTo(new Vector2d(12, 60))
                .splineToSplineHeading(new Pose2d(17, 33), Math.toRadians(270))
                .lineTo(new Vector2d(17,36))
                                .addTemporalMarker(1.9, () -> robot.rightServo.setPosition(RobotConstants.rightIn))
                .lineTo(new Vector2d(18, 36))
                .splineToConstantHeading(new Vector2d(45, 37.5), Math.toRadians(0))
                .lineTo(new Vector2d(53, 37.5))
                                .addTemporalMarker(3.9, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
                .waitSeconds(.3)
                .lineTo(new Vector2d(51, 37.5))
                .addTemporalMarker(4, () -> {
                                    robot.leftLiftServo.setPosition(RobotConstants.stack5+RobotConstants.stackLeftOffset);
                                    robot.rightLiftServo.setPosition(RobotConstants.stack5);
                                    robot.transferMotor.setPower(.3);
                                    drawbridgeTargetPos = RobotConstants.stackDrawbridgeDown;
                })
                .addTemporalMarker(5, () ->
                        targetSlidePos = RobotConstants.slideBottom)
                .addTemporalMarker(6.4, () -> {
                                    robot.intakeMotor.setPower(1);
                                    robot.transferMotor.setPower(1);
                                    robot.dropServo.setPosition(RobotConstants.dropClosed);
                })
                .splineToConstantHeading(new Vector2d(9, 8), Math.toRadians(180))
                .lineTo(new Vector2d(-20, 8))
                .splineToConstantHeading(new Vector2d(-56.2, 15), Math.toRadians(180))
                .waitSeconds(.3)
                .lineTo(new Vector2d(-50, 15))
                .waitSeconds(.3)
                .addTemporalMarker(8.2, () -> {
                                    robot.leftLiftServo.setPosition(RobotConstants.stack4+RobotConstants.stackLeftOffset);
                                    robot.rightLiftServo.setPosition(RobotConstants.stack4);
                })
                .lineTo(new Vector2d(-56.2, 15))
                .waitSeconds(.3)
                .lineTo(new Vector2d(-55, 15))
                .splineToConstantHeading(new Vector2d(-20, 8), Math.toRadians(0))
                .lineTo(new Vector2d(9, 8))
                .addTemporalMarker(11.7, () -> targetSlidePos = RobotConstants.slideLow)
                .splineToConstantHeading(new Vector2d(53, 31), Math.toRadians(0))
                .addTemporalMarker(12.5, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
                .waitSeconds(.3)
                .build();


        TrajectorySequence right = driveTrain.trajectorySequenceBuilder(new Pose2d(12, 63, Math.toRadians(270)))
                .lineTo(new Vector2d(12, 45))
                .splineToConstantHeading(new Vector2d(8.5, 36), Math.toRadians(180))
                .lineTo(new Vector2d(7, 36))
                .addTemporalMarker(1.6, () -> robot.rightServo.setPosition(RobotConstants.rightIn))
                .lineTo(new Vector2d(20, 36))
                .splineToSplineHeading(new Pose2d(45, 28.5, Math.toRadians(0)), Math.toRadians(0))
                .lineTo(new Vector2d(53, 28.5))
                .addTemporalMarker(3.7, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
                .waitSeconds(.3)
                .lineTo(new Vector2d(40, 28.5))
                .addDisplacementMarker(() -> {targetSlidePos = RobotConstants.slideBottom; robot.dropServo.setPosition(RobotConstants.dropClosed);})
                .lineTo(new Vector2d(40, 60))
                .lineTo(new Vector2d(45, 60))
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
        driveTrain.setPoseEstimate(new Pose2d(12, 63, Math.toRadians(270)));

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
            if (drawbridgeTargetPos<drawbridgeCurrentPos) {
                drawbridgeCurrentPos+= Range.clip((drawbridgeTargetPos-drawbridgeCurrentPos), -.015, -.0);
                robot.rightDrawbridgeServo.setPosition(drawbridgeCurrentPos+RobotConstants.drawbridgeRightOffset);
                robot.leftDrawbridgeServo.setPosition(drawbridgeCurrentPos);
            } else if (drawbridgeTargetPos>drawbridgeCurrentPos) {
                drawbridgeCurrentPos+=Range.clip((drawbridgeTargetPos-drawbridgeCurrentPos), 0, .015);
                robot.rightDrawbridgeServo.setPosition(drawbridgeCurrentPos+RobotConstants.drawbridgeRightOffset);
                robot.leftDrawbridgeServo.setPosition(drawbridgeCurrentPos);
            }
        }

    }
}