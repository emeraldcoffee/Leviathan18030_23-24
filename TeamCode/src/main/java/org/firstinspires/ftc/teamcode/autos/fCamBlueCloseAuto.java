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
import org.firstinspires.ftc.teamcode.pipelines.ColorMask;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "fCam Blue Close Auto")
public class fCamBlueCloseAuto extends LinearOpMode {

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

    double slideI = 0;

    String pos = "";

    @Override
    public void runOpMode() throws InterruptedException {

        RobotConfig robot = new RobotConfig(hardwareMap);
        ColorMask pipeline = new ColorMask();

//        robot.leftLiftServo.setPosition(intakePos+RobotConstants.stackLeftOffset);
//        robot.rightLiftServo.setPosition(intakePos);
        robot.transferMotor.setPower(-.2);
        robot.stackHold(false);

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


        TrajectorySequence left = robot.trajectorySequenceBuilder(new Pose2d(12, 63, Math.toRadians(270)))
                .lineTo(new Vector2d(12, 61))
                .splineToLinearHeading(new Pose2d(23, 40), Math.toRadians(270))
                .lineTo(new Vector2d(23, 48))
                .addTemporalMarker(2.15, () -> robot.rightPixelServo.setPosition(RobotConstants.rightIn))
                .lineTo(new Vector2d(26, 48))
                .splineToConstantHeading(new Vector2d(47, 44), Math.toRadians(0))
                .lineTo(new Vector2d(53, 44))
                .addTemporalMarker(3.8, () -> robot.dropper(RobotConfig.Dropper.OPEN))
                .waitSeconds(.4)
                .lineTo(new Vector2d(40, 44))
                .addDisplacementMarker(() -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                    robot.dropper(RobotConfig.Dropper.CLOSED);
                })
                .lineTo(new Vector2d(40, 12))
                .lineTo(new Vector2d(45, 12))
                .build();

        TrajectorySequence center = robot.trajectorySequenceBuilder(new Pose2d(12, 63, Math.toRadians(270)))
                .lineTo(new Vector2d(12, 60))
                .splineToSplineHeading(new Pose2d(17, 31), Math.toRadians(270))
                .lineTo(new Vector2d(17,36))
                .addTemporalMarker(1.9, () -> robot.rightPixelServo.setPosition(RobotConstants.rightIn))
                .lineTo(new Vector2d(18, 36))
                .splineToConstantHeading(new Vector2d(45, 37.5), Math.toRadians(0))
                .lineTo(new Vector2d(53, 37.5))
                .addTemporalMarker(3.9, () -> robot.dropper(RobotConfig.Dropper.OPEN))
                .waitSeconds(.3)
                .lineTo(new Vector2d(40, 37.5))
                .addDisplacementMarker(() -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                    robot.dropper(RobotConfig.Dropper.CLOSED);
                })
                .lineTo(new Vector2d(40, 12))
                .lineTo(new Vector2d(45, 12))
                .build();

        TrajectorySequence right = robot.trajectorySequenceBuilder(new Pose2d(12, 63, Math.toRadians(270)))
                .lineTo(new Vector2d(12, 45))
                .splineToConstantHeading(new Vector2d(8.5, 36), Math.toRadians(180))
                .lineTo(new Vector2d(8, 36))
                .addTemporalMarker(1.5, () -> robot.rightPixelServo.setPosition(RobotConstants.rightIn))
                .lineTo(new Vector2d(20, 36))
                .splineToSplineHeading(new Pose2d(45, 31, Math.toRadians(0)), Math.toRadians(0))
                .lineTo(new Vector2d(53, 31))
                .addTemporalMarker(4.1, () -> robot.dropper(RobotConfig.Dropper.OPEN))
                .waitSeconds(.3)
                .lineTo(new Vector2d(40, 31))
                .addDisplacementMarker(() -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                    robot.dropper(RobotConfig.Dropper.CLOSED);
                })
                .lineTo(new Vector2d(40, 12))
                .lineTo(new Vector2d(45, 12))
                .build();



        ElapsedTime cameraDelayTimer = new ElapsedTime();

        telemetry.setAutoClear(false);
        Telemetry.Item detectedPos = telemetry.addData("Position", "No detection");
        Telemetry.Item wasPos = telemetry.addData("Was pos", "");

//        Telemetry.Item slideData = telemetry.addData("Slide Data:", "Encoder Val:" + robot.liftEncoder.getCurrentPosition() + " Target Val:" + targetSlidePos);

        robot.rightPixelServo.setPosition(RobotConstants.rightOut);
        robot.dropServo.setPosition(RobotConstants.dropClosed);
        robot.ResetSlides();

        waitForStart();
        if (isStopRequested()) return;

        robot.climbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.update();
        robot.setPoseEstimate(new Pose2d(12, 63, Math.toRadians(270)));

        robot.setTargetSlidePos(RobotConfig.SlideHeight.PRELOAD_DROP);

        cameraDelayTimer.reset();

        while (opModeIsActive() && !isStopRequested()) {
//            robot.update();

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
//                    wasPos.setValue(RobotMethods.updateRobotPosition(robot.getPoseEstimate()));

                    camera = Camera.FINISHED;
                    break;
                case FINISHED:
                    break;
            }

           robot.update();

//            slideData.setValue( "Encoder Val: " + slideCurPos + " Target Val: " + targetSlidePos + " Slide Power: " + (double)Math.round(slidePower*100)/100);

        }

    }
}