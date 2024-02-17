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

        TrajectorySequence right = robot.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(90)))
                .splineToSplineHeading(new Pose2d(-36, -35, Math.toRadians(0)), Math.toRadians(90))
                .addTemporalMarker(.1, () -> {
                                robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                })
                .lineTo(new Vector2d(-23, -35))
                .addTemporalMarker(2.9, () -> {
                                robot.leftPixelServo.setPosition(RobotConstants.leftIn);
                })
                .lineTo(new Vector2d(-28, -35))
                .addTemporalMarker(3.5, () -> {
                    robot.intakeMotor.setPower(1);
                                robot.transferMotor.setPower(1);
                                robot.stackArm(RobotConfig.StackArm.FAR_OUT);
                })
                .splineToConstantHeading(new Vector2d(-57.5, -36), Math.toRadians(180))
                .addTemporalMarker(4.7, () -> {
                                robot.stackHold(true);
                })
                .addTemporalMarker(4.8, () -> {
                                robot.stackArm(RobotConfig.StackArm.IN);
                })
                .addTemporalMarker(5.1, () -> {
                                robot.stackArm(RobotConfig.StackArm.FAR_OUT);
                })
                .addTemporalMarker(5.3, () -> {
                    robot.stackHold(false);
                })
                .waitSeconds(.7)
                .forward(.5)
                .splineToConstantHeading(new Vector2d(-33, -57), Math.toRadians(0))
                .lineTo(new Vector2d(31, -57))
                .addTemporalMarker(7.7, () -> {
                                robot.setTargetSlidePos(RobotConfig.SlideHeight.PRELOAD_DROP);
                                robot.intakeMotor.setPower(0);
                                robot.transferMotor.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(50, -41.5), Math.toRadians(0))
                .lineTo(new Vector2d(53, -41.5))
                .addTemporalMarker(9.6, () -> {
                                robot.dropper(RobotConfig.Dropper.OPEN);
                })
                .addTemporalMarker(9.9, () -> {
                                robot.setTargetSlidePos(RobotConfig.SlideHeight.LOW);
                                robot.dropper(RobotConfig.Dropper.PARTIAL);
                })
                .waitSeconds(1.1)
//                            //Cycle 1
                .back(.1)
                .splineToConstantHeading(new Vector2d(33, -57), Math.toRadians(180))
                .addTemporalMarker(12.0, () -> {
                                robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                })
                .lineTo(new Vector2d(-33, -57))
                .splineToConstantHeading(new Vector2d(-57.5, -36), Math.toRadians(180))
                .addTemporalMarker(13.0, () -> {
                                robot.intakeMotor.setPower(1);
                                robot.transferMotor.setPower(1);
                                robot.dropper(RobotConfig.Dropper.CLOSED);
                })
                .waitSeconds(1)
                .addTemporalMarker(14.6, () -> {
                                robot.stackHold(true);
                })
                .addTemporalMarker(14.7, () -> {
                                robot.stackArm(RobotConfig.StackArm.IN);
                })
                .addTemporalMarker(15.0, () -> {
                                robot.stackArm(RobotConfig.StackArm.OUT);
                })
                .addTemporalMarker(15.2, () -> {
                                robot.stackArm(RobotConfig.StackArm.IN);
                })
                .addTemporalMarker(15.5, () -> {
                                robot.stackArm(RobotConfig.StackArm.FAR_OUT);
                })
                .addTemporalMarker(15.6, () -> {
                                robot.stackHold(false);
                })
                .waitSeconds(10)
                .build();

        TrajectorySequence center = robot.trajectorySequenceBuilder(new Pose2d(-35, -63, Math.toRadians(90)))
                .splineToSplineHeading(new Pose2d(-40, -32, Math.toRadians(0)), Math.toRadians(90))
                .addTemporalMarker(.1, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                })
                .lineTo(new Vector2d(-40, -33))
                .addTemporalMarker(1.7, () -> {robot.leftPixelServo.setPosition(RobotConstants.leftIn);})
                .addTemporalMarker(1.4, () -> {
                    robot.intakeMotor.setPower(1);
                    robot.transferMotor.setPower(1);
                    robot.stackArm(RobotConfig.StackArm.FAR_OUT);
                })
                .splineToConstantHeading(new Vector2d(-56, -35), Math.toRadians(180))
                .addTemporalMarker(2.9, () -> {
                    robot.stackHold(true);
                })
                .addTemporalMarker(3.0, () -> {
                    robot.stackArm(RobotConfig.StackArm.IN);
                })
                .addTemporalMarker(3.3, () -> {
                    robot.stackArm(RobotConfig.StackArm.FAR_OUT);
                })
                .addTemporalMarker(3.5, () -> {
                })
                .waitSeconds(5)
                .build();

        TrajectorySequence left = robot.trajectorySequenceBuilder(new Pose2d(-35, -63, Math.toRadians(90)))
                .splineToSplineHeading(new Pose2d(-45, -28, Math.toRadians(0)), Math.toRadians(90))
                .addTemporalMarker(.1, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                })
                .lineTo(new Vector2d(-45, -34))
                .addTemporalMarker(2.3, () -> {
                    robot.leftPixelServo.setPosition(RobotConstants.leftIn);
                })
                .addTemporalMarker(1.2, () -> {
                                    robot.intakeMotor.setPower(1);
                                    robot.transferMotor.setPower(1);
                                    robot.stackArm(RobotConfig.StackArm.FAR_OUT);
                })
                .splineToConstantHeading(new Vector2d(-56.5, -35), Math.toRadians(180))
                .addTemporalMarker(3.1, () -> {
                                    robot.stackHold(true);
                })
                .addTemporalMarker(3.2, () -> {
                    robot.stackArm(RobotConfig.StackArm.IN);
                })
                .addTemporalMarker(3.5, () -> {
                    robot.stackArm(RobotConfig.StackArm.FAR_OUT);
                })
                .addTemporalMarker(3.7, () -> {
                    robot.stackHold(false);
                })
                .waitSeconds(5)
                .build();

        ElapsedTime cameraDelayTimer = new ElapsedTime();

        telemetry.setAutoClear(false);
        Telemetry.Item detectedPos = telemetry.addData("Position", "No detection");

//        Telemetry.Item slideData = telemetry.addData("Slide Data:", "Encoder Val:" + robot.liftEncoder.getCurrentPosition() + " Target Val:" + targetSlidePos);


        robot.leftPixelServo.setPosition(RobotConstants.leftOut);
        robot.dropper(RobotConfig.Dropper.CLOSED);


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
                    camera = fCamRedFarAutoAvoid.Camera.FINISHED;
                    break;
                case FINISHED:
                    break;
            }


        }
    }
}
