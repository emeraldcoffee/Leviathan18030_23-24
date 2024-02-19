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
                    robot.stackHold(true);
                })
                .addTemporalMarker(15.8, () -> {
                    robot.stackArm(RobotConfig.StackArm.IN);
                })
                .addTemporalMarker(16.1, () -> {
                    robot.stackArm(RobotConfig.StackArm.OUT);
                })
                .addTemporalMarker(16.3, () -> {
                    robot.stackArm(RobotConfig.StackArm.IN);
                })
                .addTemporalMarker(16.6, () -> {
                    robot.stackArm(RobotConfig.StackArm.OUT);
                })
                .addTemporalMarker(16.7, () -> {
                    robot.stackHold(false);
                })
                //To backdrop
                .forward(.5)
                .splineToConstantHeading(new Vector2d(-33, -57), Math.toRadians(0))
                .lineTo(new Vector2d(31, -57))
                .addTemporalMarker(19.6, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.LOW);
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
                .addTemporalMarker(24.3, () -> {
//                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                    robot.dropper(RobotConfig.Dropper.CLOSED);
                })
                .back(6)
                .waitSeconds(1.5)
                .build();

        //3.88 stack 1, 14.68 stack 2
        TrajectorySequence center = robot.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(90)))
                .splineToSplineHeading(new Pose2d(-39, -32, Math.toRadians(0)), Math.toRadians(90))
                .addTemporalMarker(.1, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                })
                .lineTo(new Vector2d(-39, -37))
                .addTemporalMarker(1.8, () -> {
                    robot.leftPixelServo.setPosition(RobotConstants.leftIn);
                })
                .back(.1)
                .addTemporalMarker(2.2, () -> {
                    robot.intakeMotor.setPower(1);
                    robot.transferMotor.setPower(1);
                    robot.stackArm(RobotConfig.StackArm.OUT);
                })
                .splineToConstantHeading(new Vector2d(-57, -35.8), Math.toRadians(180))
                .addTemporalMarker(4.0, () -> {
                    robot.stackHold(true);
                })
                .addTemporalMarker(4.1, () -> {
                    robot.stackArm(RobotConfig.StackArm.IN);
                })
                .addTemporalMarker(4.4, () -> {
                    robot.stackArm(RobotConfig.StackArm.FAR_OUT);
                })
                .addTemporalMarker(4.6, () -> {
                    robot.stackHold(false);
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
                .splineToConstantHeading(new Vector2d(50, -36), Math.toRadians(0))
                .lineTo(new Vector2d(53, -36), RobotConfig.getVelocityConstraint(25, Math.toRadians(200), 10.62), RobotConfig.getAccelerationConstraint(25))
                .addTemporalMarker(9.2, () -> {
                    robot.dropper(RobotConfig.Dropper.OPEN);
                    robot.safeRelocalizeBackdrop();
                })
                .addTemporalMarker(9.5, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.LOW);
                    robot.dropper(RobotConfig.Dropper.PARTIAL);
                })
                .waitSeconds(1.3)
                //Cycle 1
                .back(.1)
                .splineToConstantHeading(new Vector2d(33, -57), Math.toRadians(180))
                .addTemporalMarker(11.1, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                })
                .lineTo(new Vector2d(-33, -57))
                .splineToConstantHeading(new Vector2d(-52, -37.5), Math.toRadians(110))
                .splineToConstantHeading(new Vector2d(-56.5, -36), Math.toRadians(180))
                .addTemporalMarker(13.7, () -> {
                    robot.intakeMotor.setPower(1);
                    robot.transferMotor.setPower(1);
                    robot.dropper(RobotConfig.Dropper.CLOSED);
                    robot.stackArm(RobotConfig.StackArm.FAR_LEFT);
                })
                .waitSeconds(1.7)
                .addTemporalMarker(14.4, () -> {
                    robot.stackArm(RobotConfig.StackArm.OUT);
                })
                .addTemporalMarker(14.6, () -> {
                    robot.stackHold(true);
                })
                .addTemporalMarker(14.8, () -> {
                    robot.stackArm(RobotConfig.StackArm.IN);
                })
                .addTemporalMarker(15.1, () -> {
                    robot.stackArm(RobotConfig.StackArm.OUT);
                })
                .addTemporalMarker(15.3, () -> {
                    robot.stackArm(RobotConfig.StackArm.IN);
                })
                .addTemporalMarker(15.6, () -> {
                    robot.stackArm(RobotConfig.StackArm.OUT);
                })
                .addTemporalMarker(15.7, () -> {
                    robot.stackHold(false);
                })
                //To backdrop
                .forward(.5)
                .splineToConstantHeading(new Vector2d(-33, -57), Math.toRadians(0))
                .lineTo(new Vector2d(31, -57))
                .addTemporalMarker(19.1, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.LOW);
                    robot.intakeMotor.setPower(0);
                    robot.transferMotor.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(50, -41), Math.toRadians(0))
                .lineTo(new Vector2d(53, -41), RobotConfig.getVelocityConstraint(25, Math.toRadians(200), 10.62), RobotConfig.getAccelerationConstraint(25))
                .addTemporalMarker(20.9, () -> {
                    robot.dropper(RobotConfig.Dropper.PARTIAL);
                    robot.safeRelocalizeBackdrop();
                })
                .waitSeconds(1)
                .addTemporalMarker(22.3, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                    robot.dropper(RobotConfig.Dropper.CLOSED);
                })
                .back(6)
                .waitSeconds(1)
                .build();

        //3.38 stack 1, 14.46 stack 2
        TrajectorySequence left = robot.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(90)))
                .splineToSplineHeading(new Pose2d(-45, -28, Math.toRadians(0)), Math.toRadians(90))
                .addTemporalMarker(.1, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                })
                .lineTo(new Vector2d(-45, -31))
                .addTemporalMarker(2.1, () -> {
                    robot.leftPixelServo.setPosition(RobotConstants.leftIn);
                })
                .addTemporalMarker(2.4, () -> {
                    robot.intakeMotor.setPower(1);
                    robot.transferMotor.setPower(1);
                    robot.stackArm(RobotConfig.StackArm.OUT);
                })
                .splineToConstantHeading(new Vector2d(-57, -35.8), Math.toRadians(180))
                .addTemporalMarker(3.5, () -> {
                    robot.stackHold(true);
                })
                .addTemporalMarker(3.6, () -> {
                    robot.stackArm(RobotConfig.StackArm.IN);
                })
                .addTemporalMarker(3.9, () -> {
                    robot.stackArm(RobotConfig.StackArm.FAR_OUT);
                })
                .addTemporalMarker(4.1, () -> {
                    robot.stackHold(false);
                })
                .waitSeconds(.9)
                .forward(.5)
                .splineToConstantHeading(new Vector2d(-33, -57), Math.toRadians(0))
                .lineTo(new Vector2d(31, -57))
                .addTemporalMarker(6.7, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.PRELOAD_DROP);
                    robot.intakeMotor.setPower(0);
                    robot.transferMotor.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(50, -30), Math.toRadians(0))
                .lineTo(new Vector2d(53, -30), RobotConfig.getVelocityConstraint(25, Math.toRadians(200), 10.62), RobotConfig.getAccelerationConstraint(25))
                .addTemporalMarker(9.1, () -> {
                    robot.dropper(RobotConfig.Dropper.OPEN);
                    robot.safeRelocalizeBackdrop();
                })
                .addTemporalMarker(9.4, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.LOW);
                    robot.dropper(RobotConfig.Dropper.PARTIAL);
                })
                .waitSeconds(1.3)
                //Cycle 1
                .back(.1)
                .splineToConstantHeading(new Vector2d(33, -57), Math.toRadians(180))
                .addTemporalMarker(11.1, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                })
                .lineTo(new Vector2d(-33, -57))
                .splineToConstantHeading(new Vector2d(-52, -37.5), Math.toRadians(110))
                .splineToConstantHeading(new Vector2d(-56.5, -36), Math.toRadians(180))
                .addTemporalMarker(13.5, () -> {
                    robot.intakeMotor.setPower(1);
                    robot.transferMotor.setPower(1);
                    robot.dropper(RobotConfig.Dropper.CLOSED);
                    robot.stackArm(RobotConfig.StackArm.FAR_LEFT);
                })
                .waitSeconds(1.7)
                .addTemporalMarker(14.2, () -> {
                    robot.stackArm(RobotConfig.StackArm.OUT);
                })
                .addTemporalMarker(14.4, () -> {
                    robot.stackHold(true);
                })
                .addTemporalMarker(14.6, () -> {
                    robot.stackArm(RobotConfig.StackArm.IN);
                })
                .addTemporalMarker(14.9, () -> {
                    robot.stackArm(RobotConfig.StackArm.OUT);
                })
                .addTemporalMarker(15.1, () -> {
                    robot.stackArm(RobotConfig.StackArm.IN);
                })
                .addTemporalMarker(15.4, () -> {
                    robot.stackArm(RobotConfig.StackArm.OUT);
                })
                .addTemporalMarker(15.5, () -> {
                    robot.stackHold(false);
                })
                //To backdrop
                .forward(.5)
                .splineToConstantHeading(new Vector2d(-33, -57), Math.toRadians(0))
                .lineTo(new Vector2d(31, -57))
                .addTemporalMarker(18.5, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.LOW);
                    robot.intakeMotor.setPower(0);
                    robot.transferMotor.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(50, -41), Math.toRadians(0))
                .lineTo(new Vector2d(53, -41), RobotConfig.getVelocityConstraint(25, Math.toRadians(200), 10.62), RobotConfig.getAccelerationConstraint(25))
                .addTemporalMarker(21, () -> {
                    robot.dropper(RobotConfig.Dropper.PARTIAL);
                    robot.safeRelocalizeBackdrop();
                })
                .waitSeconds(1)
                .addTemporalMarker(22.3, () -> {
                    robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                    robot.dropper(RobotConfig.Dropper.CLOSED);
                })
                .back(6)
                .waitSeconds(1)
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
                    camera = fCamRedFarAutoAvoid.Camera.FINISHED;
                    break;
                case FINISHED:
                    break;
            }


        }
    }
}
