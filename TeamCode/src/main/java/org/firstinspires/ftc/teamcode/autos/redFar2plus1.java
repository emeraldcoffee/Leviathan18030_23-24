package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.RobotConfig;
import org.firstinspires.ftc.teamcode.pipelines.ColorMask;
import org.firstinspires.ftc.teamcode.robot.PassData;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
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

    Camera camera = Camera.WAIT;

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

        robot.ResetSlides();

        robot.transferMotor.setPower(-.2);
//        robot.spikeMarkHoldServo.setPosition(RobotConstants.holdServoUp);
        robot.stackHold(false);

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        robot.webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"));

        robot.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                pipeline.setStart("far");
                pipeline.setAlliance("Red");
                robot.webcam.setPipeline(pipeline);

                robot.webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error: ", errorCode);
            }
        });

        //4.57 stack 1, 15.09 stack 2
        TrajectorySequence right = robot.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(90)))
                .splineToSplineHeading(new Pose2d(-36, -35, Math.toRadians(0)), Math.toRadians(90))
                .addTemporalMarker(.1, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                })
                .lineTo(new Vector2d(-24, -35))
                .addTemporalMarker(2.7, () -> {
                    robot.leftPixelServo.setPosition(RobotConstants.leftIn);
                })
                .lineTo(new Vector2d(-28, -35))
                .addTemporalMarker(3.5, () -> {
                    robot.intakeMotor.setPower(1);
                    robot.transferMotor.setPower(1);
                    robot.stackArm(RobotConfig.StackArm.OUT);
                })
                .splineToConstantHeading(new Vector2d(-57, -35.5), Math.toRadians(180))
                .addTemporalMarker(4.7, () -> {
                    robot.grabFromStack(1);
                })
                .waitSeconds(.9)
                .forward(.5)
                .splineToConstantHeading(new Vector2d(-33, -57), Math.toRadians(0))
                .lineTo(new Vector2d(31, -57))
                .addTemporalMarker(7.9, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.PRELOAD_DROP);
                    robot.intakeMotor.setPower(0);
                    robot.transferMotor.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(50, -42), Math.toRadians(0))
                .lineTo(new Vector2d(53, -42), RobotConfig.getVelocityConstraint(25, Math.toRadians(200), 10.62), RobotConfig.getAccelerationConstraint(25))
                .addTemporalMarker(10.1, () -> {
                    robot.dropper(RobotConfig.Dropper.OPEN);
                    robot.safeRelocalizeBackdrop();
                })
                .addTemporalMarker(10.4, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.LOW);
                    robot.dropper(RobotConfig.Dropper.PARTIAL);
                })
                .waitSeconds(1.3)
//                            //Cycle 1
                .back(.1)
                .splineToConstantHeading(new Vector2d(33, -57), Math.toRadians(180))
                .addTemporalMarker(12.2, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                })
                .lineTo(new Vector2d(-33, -57))
                .splineToConstantHeading(new Vector2d(-52, -37.5), Math.toRadians(110))
                .splineToConstantHeading(new Vector2d(-56.5, -35), Math.toRadians(180))
                .addTemporalMarker(14.2, () -> {
                    robot.intakeMotor.setPower(1);
                    robot.transferMotor.setPower(1);
                    robot.dropper(RobotConfig.Dropper.CLOSED);
                    robot.stackArm(RobotConfig.StackArm.FAR_LEFT);
                })
                .waitSeconds(1.7)
                .addTemporalMarker(15.2, () -> {
                    robot.stackArm(RobotConfig.StackArm.OUT);
                })
                .addTemporalMarker(15.6, () -> {
                    robot.grabFromStack(2);
                })
                //To backdrop
                .forward(.5)
                .splineToConstantHeading(new Vector2d(-33, -57), Math.toRadians(0))
                .lineTo(new Vector2d(31, -57))
                .addTemporalMarker(19.6, () -> {
                    robot.setTargetSlidePos(17);
                    robot.intakeMotor.setPower(0);
                    robot.transferMotor.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(50, -41), Math.toRadians(0))
                .lineTo(new Vector2d(53, -41), RobotConfig.getVelocityConstraint(25, Math.toRadians(200), 10.62), RobotConfig.getAccelerationConstraint(25))
                .addTemporalMarker(22.3, () -> {
                    robot.dropper(RobotConfig.Dropper.PARTIAL);
                    robot.safeRelocalizeBackdrop();
                })
                .waitSeconds(2)
                .addTemporalMarker(25, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                    robot.dropper(RobotConfig.Dropper.CLOSED);
                })
                .lineTo(new Vector2d(47, -41))
                .lineTo(new Vector2d(47, PassData.roadrunnerParkPosition.red))
                .build();

        //3.88 stack 1, 14.68 stack 2
        TrajectorySequence center = robot.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(90)))
                .splineToSplineHeading(new Pose2d(-39, -32.4, Math.toRadians(0)), Math.toRadians(90))
                .addTemporalMarker(.1, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                })
                .lineTo(new Vector2d(-39.5, -37))
                .addTemporalMarker(1.5, () -> {
                    robot.leftPixelServo.setPosition(RobotConstants.leftIn);
                })
                .back(.1)
                .addTemporalMarker(2.2, () -> {
                    robot.intakeMotor.setPower(1);
                    robot.transferMotor.setPower(1);
                    robot.stackArm(RobotConfig.StackArm.OUT);
                })
                .splineToConstantHeading(new Vector2d(-57.5, -35.7), Math.toRadians(180))
                .addTemporalMarker(4.0, () -> {
                    robot.grabFromStack(1);
                })
                .waitSeconds(.9)
                .forward(.5)
                .splineToConstantHeading(new Vector2d(-33, -57), Math.toRadians(0))
                .lineTo(new Vector2d(31, -57))
                .addTemporalMarker(7.2, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.PRELOAD_DROP);
                    robot.intakeMotor.setPower(0);
                    robot.transferMotor.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(50, -35.5), Math.toRadians(0))
                .lineTo(new Vector2d(53, -35.5), RobotConfig.getVelocityConstraint(25, Math.toRadians(200), 10.62), RobotConfig.getAccelerationConstraint(25))
                .addTemporalMarker(9.2, () -> {
                    robot.dropper(RobotConfig.Dropper.OPEN);
                    robot.safeRelocalizeBackdrop();
                })
                .addTemporalMarker(9.7, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.LOW);
                    robot.dropper(RobotConfig.Dropper.PARTIAL);
                })
                .waitSeconds(1.3)
                //Cycle 1
                .back(.1)
                .splineToConstantHeading(new Vector2d(33, -57), Math.toRadians(180))
                .addTemporalMarker(11.3, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                })
                .lineTo(new Vector2d(-33, -57))
                .splineToConstantHeading(new Vector2d(-52, -37.5), Math.toRadians(110))
                .splineToConstantHeading(new Vector2d(-56.4, -34.7), Math.toRadians(180))
                .addTemporalMarker(13.7, () -> {
                    robot.intakeMotor.setPower(1);
                    robot.transferMotor.setPower(1);
                    robot.dropper(RobotConfig.Dropper.CLOSED);
                    robot.stackArm(RobotConfig.StackArm.FAR_LEFT);
                })
                .waitSeconds(1.7)
                .addTemporalMarker(15.0, () -> {
                    robot.stackArm(RobotConfig.StackArm.OUT);
                })
                .addTemporalMarker(15.3, () -> {
                    robot.grabFromStack(2);
                })
                //To backdrop
                .forward(.5)
                .splineToConstantHeading(new Vector2d(-33, -57), Math.toRadians(0))
                .lineTo(new Vector2d(31, -57))
                .addTemporalMarker(19.1, () -> {
                    robot.setTargetSlidePos(17);
                    robot.intakeMotor.setPower(0);
                    robot.transferMotor.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(50, -37), Math.toRadians(0))
                .lineTo(new Vector2d(53, -37), RobotConfig.getVelocityConstraint(25, Math.toRadians(200), 10.62), RobotConfig.getAccelerationConstraint(25))
                .addTemporalMarker(21.7, () -> {
                    robot.dropper(RobotConfig.Dropper.PARTIAL);
                    robot.safeRelocalizeBackdrop();
                })
                .waitSeconds(1)
                .addTemporalMarker(23.5, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                    robot.dropper(RobotConfig.Dropper.CLOSED);
                })
                .lineTo(new Vector2d(47, -37))
                .lineTo(new Vector2d(47, PassData.roadrunnerParkPosition.red))
                .build();

        //3.38 stack 1, 14.46 stack 2
        TrajectorySequence left = robot.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(-51, -35), Math.toRadians(90))
                .addTemporalMarker(.1, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                })
                .splineToSplineHeading(new Pose2d(-46.5, -34), Math.toRadians(-30))
                .waitSeconds(.4)
                .addTemporalMarker(2.8, () -> {
                    robot.leftPixelServo.setPosition(RobotConstants.leftIn);
                })
                .addTemporalMarker(2.9, () -> {
                    robot.intakeMotor.setPower(1);
                    robot.transferMotor.setPower(1);
                    robot.stackArm(RobotConfig.StackArm.OUT);
                })
                .back(.1)
                .splineToConstantHeading(new Vector2d(-57, -35.4), Math.toRadians(180))
                .addTemporalMarker(3.7, () -> {
                    robot.grabFromStack(1);
                })
                .waitSeconds(.9)
                .forward(.5)
                .splineToConstantHeading(new Vector2d(-33, -57), Math.toRadians(0))
                .lineTo(new Vector2d(31, -57))
                .addTemporalMarker(7.1, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.PRELOAD_DROP);
                    robot.intakeMotor.setPower(0);
                    robot.transferMotor.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(50, -29), Math.toRadians(0))
                .lineTo(new Vector2d(52.4, -29), RobotConfig.getVelocityConstraint(25, Math.toRadians(200), 10.62), RobotConfig.getAccelerationConstraint(25))
                .addTemporalMarker(10.0, () -> {
                    robot.dropper(RobotConfig.Dropper.OPEN);
                    robot.safeRelocalizeBackdrop();
                })
                .addTemporalMarker(10.4, () -> {
                    robot.setTargetSlidePos(17);
                    robot.dropper(RobotConfig.Dropper.PARTIAL);
                })
                .waitSeconds(1.2)
                //Cycle 1
                .back(.1)
                .splineToConstantHeading(new Vector2d(33, -57), Math.toRadians(180))
                .addTemporalMarker(12.4, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                })
                .lineTo(new Vector2d(-33, -57))
                .splineToConstantHeading(new Vector2d(-52, -36), Math.toRadians(110))
                .splineToConstantHeading(new Vector2d(-56.5, -33.45), Math.toRadians(180))
                .addTemporalMarker(14.2, () -> {
                    robot.intakeMotor.setPower(1);
                    robot.transferMotor.setPower(1);
                    robot.dropper(RobotConfig.Dropper.CLOSED);
                    robot.stackArm(RobotConfig.StackArm.FAR_LEFT);
                })
                .waitSeconds(1.7)
                .addTemporalMarker(15.35, () -> {
                    robot.stackArm(RobotConfig.StackArm.OUT);
                })
                .addTemporalMarker(16, () -> {
                    robot.grabFromStack(2);
                })
                //To backdrop

                .forward(.5)
                .splineToConstantHeading(new Vector2d(-33, -57), Math.toRadians(0))
                .lineTo(new Vector2d(31, -57))
                .addTemporalMarker(20.8, () -> {
                    robot.setTargetSlidePos(17);
                    robot.intakeMotor.setPower(0);
                    robot.transferMotor.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(50, -37), Math.toRadians(0))
                .lineTo(new Vector2d(52.3, -37), RobotConfig.getVelocityConstraint(25, Math.toRadians(200), 10.62), RobotConfig.getAccelerationConstraint(25))
                .addTemporalMarker(22.2, () -> {
                    robot.dropper(RobotConfig.Dropper.PARTIAL);
                    robot.safeRelocalizeBackdrop();
                })
                .waitSeconds(1)
                .addTemporalMarker(24, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                    robot.dropper(RobotConfig.Dropper.CLOSED);
                })
                .lineTo(new Vector2d(47, -37))
                .lineTo(new Vector2d(47, PassData.roadrunnerParkPosition.red))
                .build();

        ElapsedTime cameraDelayTimer = new ElapsedTime();

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
            IMU.setValue(robot.getCurrentIMU().toString());
            telemetry.update();

            switch (camera) {
                case WAIT:
                    if (cameraDelayTimer.seconds() > .8 + 2) {
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
