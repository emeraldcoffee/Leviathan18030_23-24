package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
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
import org.firstinspires.ftc.teamcode.robot.PassData;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous
public class redClose2Plus4 extends LinearOpMode {
    enum Camera {
        WAIT,
        SAVE,
        FIRST_DROP,
        FINISHED
    }

    Camera camera = Camera.WAIT;
    Telemetry.Item backdropReading1;
    Telemetry.Item backdropReading2;



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
                pipeline.setStart("close");
                pipeline.setAlliance("Red");
                robot.webcam.setPipeline(pipeline);

//                robot.webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
                robot.webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
                FtcDashboard.getInstance().startCameraStream(robot.webcam, 10);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error: ", errorCode);
            }
        });

        Telemetry.Item parkPos = telemetry.addData("Park Position", PassData.roadrunnerParkPosition.toString());

        boolean selecting = true;
        while (selecting) {
            if (gamepad1.x) {
                PassData.roadrunnerParkPosition = RobotConstants.ParkPosition.WALL;
            } else if (gamepad1.b) {
                PassData.roadrunnerParkPosition = RobotConstants.ParkPosition.CENTER;
            }

            if (gamepad1.back) {
                selecting = false;
            }

            parkPos.setValue(PassData.roadrunnerParkPosition.toString());
            telemetry.update();
        }

        //4.57 stack 1, 15.09 stack 2
        TrajectorySequence right = robot.trajectorySequenceBuilder(new Pose2d(16.7, -62, Math.toRadians(90)))
                .lineTo(new Vector2d(16.7, -61))
                .splineToLinearHeading(new Pose2d(22, -40), Math.toRadians(90))
                .lineTo(new Vector2d(22, -47))
                .addTemporalMarker(2.0, () -> robot.leftPixelServo.setPosition(RobotConstants.leftIn))
                .lineTo(new Vector2d(26, -47))
                .splineToConstantHeading(new Vector2d(47, -42.7), Math.toRadians(0))
                .lineTo(new Vector2d(51.7, -42.7))
                .addTemporalMarker(4.5, () -> {
                    robot.dropper(RobotConfig.Dropper.OPEN);
                    backdropReading1.setValue(robot.relocalizeRight());
                })
                .waitSeconds(.3)
                .back(.1)
                .addTemporalMarker(5.2, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                    robot.dropper(RobotConfig.Dropper.CLOSED);
                    robot.transferMotor.setPower(.3);
                })
                //cycle 1
                .splineToConstantHeading(new Vector2d(24, -11.9), Math.toRadians(180))
                .addTemporalMarker(8.0, () -> {
                    robot.intakeMotor.setPower(1);
                    robot.transferMotor.setPower(1);
                    robot.stackArm(RobotConfig.StackArm.OUT);
                })
                .lineTo(new Vector2d(-59.15, -11.9))
                .waitSeconds(1)
                .addTemporalMarker(8.5, () -> {
                    robot.grabFromStack(2);
                })
                .build();

        //3.88 stack 1, 14.68 stack 2
        TrajectorySequence center = robot.trajectorySequenceBuilder(new Pose2d(16.7, -62, Math.toRadians(90)))
                .lineTo(new Vector2d(16.7, -60))
                .splineToSplineHeading(new Pose2d(16, -32), Math.toRadians(90))
                .lineTo(new Vector2d(16,-36))
                .addTemporalMarker(1.8, () -> robot.leftPixelServo.setPosition(RobotConstants.leftIn))
                .lineTo(new Vector2d(18, -36))
                .splineToConstantHeading(new Vector2d(45, -36.65), Math.toRadians(0))
                .lineTo(new Vector2d(51.5, -36.65))
                .addTemporalMarker(4.35, () -> {
                    robot.dropper(RobotConfig.Dropper.OPEN);
                    backdropReading1.setValue(robot.relocalizeRight());
                })
                .waitSeconds(.3)
                .back(.1)
                .addTemporalMarker(5.2, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                    robot.dropper(RobotConfig.Dropper.CLOSED);
                    robot.transferMotor.setPower(.3);
                })
                //cycle 1
                .splineToConstantHeading(new Vector2d(24, -11.9), Math.toRadians(180))
                .addTemporalMarker(8.0, () -> {
                    robot.intakeMotor.setPower(1);
                    robot.transferMotor.setPower(1);
                    robot.stackArm(RobotConfig.StackArm.OUT);
                })
                .lineTo(new Vector2d(-59.15, -11.9))
                .waitSeconds(1)
                .addTemporalMarker(8.7, () -> {
                    robot.grabFromStack(2);
                })
                .build();

        //3.38 stack 1, 14.46 stack 2
        TrajectorySequence left = robot.trajectorySequenceBuilder(new Pose2d(16.7, -62, Math.toRadians(90)))
                .lineTo(new Vector2d(16.7, -45))
                .splineToConstantHeading(new Vector2d(9, -34), Math.toRadians(180))
                .lineTo(new Vector2d(8.5, -34))
                .addTemporalMarker(1.5, () -> robot.leftPixelServo.setPosition(RobotConstants.leftIn))
                .lineTo(new Vector2d(20, -34))
                .splineToSplineHeading(new Pose2d(45, -27.6, Math.toRadians(0)), Math.toRadians(0))
                .lineTo(new Vector2d(53.2, -27.6))
                .addTemporalMarker(4.5, () -> {
                    robot.dropper(RobotConfig.Dropper.OPEN);
                    backdropReading1.setValue(robot.isRightReading());
//                    robot.setPoseEstimate(robot.getPoseEstimate().plus(new Pose2d(-.8, 0, 0)));
                })
                .waitSeconds(.3)
                .back(.1)
                .addTemporalMarker(5.2, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                    robot.dropper(RobotConfig.Dropper.CLOSED);
                    robot.transferMotor.setPower(.3);
                })
                //cycle 1
                .splineToConstantHeading(new Vector2d(24, -11.9), Math.toRadians(180))
                .addTemporalMarker(7.5, () -> {
                    robot.intakeMotor.setPower(1);
                    robot.transferMotor.setPower(1);
                    robot.stackArm(RobotConfig.StackArm.OUT);
                })
                .lineTo(new Vector2d(-59.15, -11.9))
                .waitSeconds(1)
                .addTemporalMarker(8.25, () -> {
                    robot.grabFromStack(2);
                })
                .build();

        TrajectorySequence cycles = robot.trajectorySequenceBuilder(center.end())
                .lineTo(new Vector2d(24, -11.9))
                .addTemporalMarker(2.2, () -> {
                    robot.setTargetSlidePos(15.2);
                    robot.intakeMotor.setPower(0);
                    robot.transferMotor.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(50, -30), Math.toRadians(0))
                .lineTo(new Vector2d(51.2, -30))
                .waitSeconds(.5)
                .addTemporalMarker(4.3, () -> {
                    robot.dropper(RobotConfig.Dropper.PARTIAL);
                    backdropReading2.setValue(robot.relocalizeRight());
                })
                .back(.1)
                .addTemporalMarker(5.5, () -> {//8.5
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                    robot.dropper(RobotConfig.Dropper.CLOSED);
                })
                //cycle 2
                .splineToConstantHeading(new Vector2d(24, -12.25), Math.toRadians(180))
                .addTemporalMarker(7.7, () -> {
                    robot.intakeMotor.setPower(1);
                    robot.transferMotor.setPower(1);
                    robot.stackArm(RobotConfig.StackArm.OUT);
                })
                .lineTo(new Vector2d(-58.56, -12.25))
                .waitSeconds(1)
                .addTemporalMarker(8.7, () -> {
                    robot.grabFromStack(2);
                })
                .lineTo(new Vector2d(24, -12.25))
                .addTemporalMarker(12.5, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.LOW);
                    robot.intakeMotor.setPower(0);
                    robot.transferMotor.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(50, -32), Math.toRadians(0))
                .lineTo(new Vector2d(51.3, -32))
                .waitSeconds(.5)
                .back(8)
                .addTemporalMarker(13.6, () -> {
                    robot.dropper(RobotConfig.Dropper.PARTIAL);
                })
                .addTemporalMarker(14.3, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                })
                .lineTo(new Vector2d(42, PassData.roadrunnerParkPosition.red))
                .build();



        ElapsedTime cameraDelayTimer = new ElapsedTime();

        telemetry.setAutoClear(false);
        Telemetry.Item detectedPos = telemetry.addData("Position", "No detection");
        backdropReading1 = telemetry.addData("Backdrop data 1", "");
        backdropReading2 = telemetry.addData("Backdrop data 2", "");


//        Telemetry.Item slideData = telemetry.addData("Slide Data:", "Encoder Val:" + robot.liftEncoder.getCurrentPosition() + " Target Val:" + targetSlidePos);


        robot.leftPixelServo.setPosition(RobotConstants.leftOut);
        robot.dropper(RobotConfig.Dropper.CLOSED);

        robot.ResetSlides();

        waitForStart();
        if (isStopRequested()) return;

        robot.setTargetSlidePos(11);//RobotConfig.SlideHeight.PRELOAD_DROP

        robot.update();
        robot.setPoseEstimate(new Pose2d(16.7, -62, Math.toRadians(90)));

//        robot.webcam.resumeViewport();

        cameraDelayTimer.reset();

        while (!isStopRequested()) {
            robot.update();

            switch (camera) {
                case WAIT:
                    if (cameraDelayTimer.seconds() > 2) {//.8
                        camera = Camera.SAVE;
                    }
                    break;
                case SAVE:
                    pos = pipeline.getPos();
                    switch (pos) {//pos
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
                    camera = Camera.FIRST_DROP;
                    break;
                case FIRST_DROP:
                    if (!robot.isBusy()) {
                        robot.followTrajectorySequenceAsync(cycles);
                        camera = Camera.FINISHED;
                    }
                    break;
                case FINISHED:
                    break;
            }

            telemetry.update();

        }
    }
}
