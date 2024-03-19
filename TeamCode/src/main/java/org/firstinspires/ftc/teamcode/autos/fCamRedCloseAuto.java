package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.autos.old.fCamBlueFarAuto;
import org.firstinspires.ftc.teamcode.drive.RobotConfig;
import org.firstinspires.ftc.teamcode.pipelines.ColorMask;
import org.firstinspires.ftc.teamcode.robot.PassData;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "fCam Red Close Auto")
public class fCamRedCloseAuto extends LinearOpMode {

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

//    int targetSlidePos = RobotConstants.slideAuto;
//
//    double intakePos = RobotConstants.stackMax;
//
//    double slideI = 0;

    String pos = "";

    @Override
    public void runOpMode() throws InterruptedException {

        RobotConfig robot = new RobotConfig(hardwareMap);
        ColorMask pipeline = new ColorMask();

        telemetry.setAutoClear(false);
        Telemetry.Item detectedPos = telemetry.addData("Position", "No detection");
        Telemetry.Item IMU = telemetry.addData("Current IMU", robot.getCurrentIMU().toString());
        Telemetry.Item Park = telemetry.addData("Park Position", PassData.roadrunnerParkPosition.toString());
        telemetry.update();

        robot.transferMotor.setPower(-.2);
        robot.stackHold(false);

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        robot.webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"));

        robot.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            public void onOpened()
            {
                pipeline.setStart("close");
                pipeline.setAlliance("Red");
                robot.webcam.setPipeline(pipeline);

                robot.webcam.startStreaming(640,480, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error: ", errorCode);
            }
        });


        TrajectorySequence left = robot.trajectorySequenceBuilder(new Pose2d(16.7, -63, Math.toRadians(90)))
                .lineTo(new Vector2d(16.7, -61))
                .splineToLinearHeading(new Pose2d(22, -40), Math.toRadians(90))
                .lineTo(new Vector2d(22, -47))
                .addTemporalMarker(2.0, () -> robot.leftPixelServo.setPosition(RobotConstants.leftIn))
                .lineTo(new Vector2d(26, -47))
                .splineToConstantHeading(new Vector2d(47, -42.7), Math.toRadians(0))
                .lineTo(new Vector2d(51.5, -42.7))
                .addTemporalMarker(4.5, () -> {
                    robot.dropper(RobotConfig.Dropper.OPEN);
                })
                .waitSeconds(.3)
                .lineTo(new Vector2d(39.5, -30))
                .addDisplacementMarker(() -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                    robot.dropper(RobotConfig.Dropper.CLOSED);
                })
                .lineTo(new Vector2d(40, PassData.roadrunnerParkPosition.red))
                .lineTo(new Vector2d(45, PassData.roadrunnerParkPosition.red))
                .build();

        TrajectorySequence center = robot.trajectorySequenceBuilder(new Pose2d(16.7, -63, Math.toRadians(90)))
                .lineTo(new Vector2d(16.7, -60))
                .splineToSplineHeading(new Pose2d(16, -32), Math.toRadians(90))
                .lineTo(new Vector2d(16,-36))
                .addTemporalMarker(1.8, () -> robot.leftPixelServo.setPosition(RobotConstants.leftIn))
                .lineTo(new Vector2d(18, -36))
                .splineToConstantHeading(new Vector2d(45, -36.65), Math.toRadians(0))
                .lineTo(new Vector2d(51.5, -36.65))
                .addTemporalMarker(4.35, () -> {
                    robot.dropper(RobotConfig.Dropper.OPEN);
                })
                .waitSeconds(.3)
                .lineTo(new Vector2d(40, -37.5))
                .addDisplacementMarker(() -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                    robot.dropper(RobotConfig.Dropper.CLOSED);
                })
                .lineTo(new Vector2d(40, PassData.roadrunnerParkPosition.red))
                .lineTo(new Vector2d(45, PassData.roadrunnerParkPosition.red))
                .build();

        TrajectorySequence right = robot.trajectorySequenceBuilder(new Pose2d(16.7, -63, Math.toRadians(90)))
                .lineTo(new Vector2d(16.7, -45))
                .splineToConstantHeading(new Vector2d(9, -34), Math.toRadians(180))
                .lineTo(new Vector2d(8.5, -34))
                .addTemporalMarker(1.5, () -> robot.leftPixelServo.setPosition(RobotConstants.leftIn))
                .lineTo(new Vector2d(20, -34))
                .splineToSplineHeading(new Pose2d(45, -27.6, Math.toRadians(0)), Math.toRadians(0))
                .lineTo(new Vector2d(53.2, -27.6))
                .addTemporalMarker(4.5, () -> {
                    robot.dropper(RobotConfig.Dropper.OPEN);
                })
                .waitSeconds(.3)
                .lineTo(new Vector2d(40, -43.9))
                .addDisplacementMarker(() -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                    robot.dropper(RobotConfig.Dropper.CLOSED);
                })
                .lineTo(new Vector2d(40, PassData.roadrunnerParkPosition.red))
                .lineTo(new Vector2d(45, PassData.roadrunnerParkPosition.red))
                .build();



        ElapsedTime cameraDelayTimer = new ElapsedTime();

        robot.leftPixelServo.setPosition(RobotConstants.leftOut);
        robot.dropServo.setPosition(RobotConstants.dropClosed);

        waitForStart();
        if (isStopRequested()) return;

        robot.climbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.ResetSlides();

        robot.update();
        robot.setPoseEstimate(new Pose2d(16.7, -63, Math.toRadians(90)));

        cameraDelayTimer.reset();
        robot.setTargetSlidePos(RobotConfig.SlideHeight.PRELOAD_DROP);

        while (opModeIsActive() && !isStopRequested()) {
            robot.update();
            IMU.setValue(robot.getCurrentIMU().toString());
            telemetry.update();

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

                    camera = Camera.FINISHED;
                    break;
                case FINISHED:
                    break;
            }

        }

    }
}