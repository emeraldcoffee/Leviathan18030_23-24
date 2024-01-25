package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name = "Red Far 2+1")
public class redFar2plus1 extends LinearOpMode {
    enum Camera {
        WAIT,
        SAVE,
        FINISHED
    }

    fCamRedFarAutoAvoid.Camera camera = fCamRedFarAutoAvoid.Camera.WAIT;

    int targetSlidePos = RobotConstants.slideAuto;

    double intakePos = RobotConstants.stackMax;

    double slideI = 0;

    String pos = "";

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);
        ColorMask pipeline = new ColorMask();
        HwMap robot = new HwMap();
        robot.init(hardwareMap);

        robot.transferMotor.setPower(-.2);

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

        TrajectorySequence right = driveTrain.trajectorySequenceBuilder(new Pose2d(-35, -63, Math.toRadians(90)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(310), 10.62))
                .splineToSplineHeading(new Pose2d(-33, -35, Math.toRadians(0)), Math.toRadians(90))
                .addTemporalMarker(.1, () -> targetSlidePos = RobotConstants.slideBottom)
                .lineTo(new Vector2d(-20, -35))
                .addTemporalMarker(2.4, () -> robot.leftServo.setPosition(RobotConstants.leftIn))
                .lineTo(new Vector2d(-21, -35))
                .addTemporalMarker(3.3, () -> {
                    robot.intakeMotor.setPower(1);
                    robot.transferMotor.setPower(1);
                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack);
                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack+RobotConstants.rightSpikeOffset);

                })
                .splineToConstantHeading(new Vector2d(-54, -35), Math.toRadians(180))
                .addTemporalMarker(4.1, () -> {
                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn+RobotConstants.rightSpikeOffset);
                })
                .addTemporalMarker(4.4, () -> {
                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack);
                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack+RobotConstants.rightSpikeOffset);
                })
                .waitSeconds(.5)
                .lineTo(new Vector2d(-53, -35))
                .splineToConstantHeading(new Vector2d(-33, -58), Math.toRadians(0))
                .lineTo(new Vector2d(33, -58))
                .addTemporalMarker(7.2, () -> {
                    targetSlidePos = RobotConstants.slideAuto;
                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide);
                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide+RobotConstants.rightSpikeOffset);
                    robot.intakeMotor.setPower(0);
                    robot.transferMotor.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(52, -41), Math.toRadians(0))
                .addTemporalMarker(8.3, () -> robot.dropServo.setPosition(RobotConstants.dropPartial))
                .waitSeconds(.3)
                .addTemporalMarker(9.6, () -> targetSlidePos = RobotConstants.slideBottom)
                .lineTo(new Vector2d(45, -41))
                .lineTo(new Vector2d(45, -61))
                .lineTo(new Vector2d(55, -61))
                .build();

        TrajectorySequence center = driveTrain.trajectorySequenceBuilder(new Pose2d(-35, -63, Math.toRadians(90)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(310), 10.62))
                .splineToSplineHeading(new Pose2d(-40, -32, Math.toRadians(0)), Math.toRadians(90))
                .addTemporalMarker(.1, () -> targetSlidePos = RobotConstants.slideBottom)
                .lineTo(new Vector2d(-40, -33))
                .addTemporalMarker(1.7, () -> {robot.leftServo.setPosition(RobotConstants.leftIn);})
                .addTemporalMarker(1.6, () -> {
                                    robot.intakeMotor.setPower(1);
                                    robot.transferMotor.setPower(1);
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack);
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack+RobotConstants.rightSpikeOffset);

                })
                .splineToConstantHeading(new Vector2d(-54, -35), Math.toRadians(180))
                .addTemporalMarker(2.9, () -> {
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn+RobotConstants.rightSpikeOffset);
                })
                .addTemporalMarker(3.2, () -> {
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack);
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack+RobotConstants.rightSpikeOffset);
                })
                .waitSeconds(.5)
                .lineTo(new Vector2d(-53, -35))
                .splineToConstantHeading(new Vector2d(-33, -58), Math.toRadians(0))
                .lineTo(new Vector2d(33, -58))
                .addTemporalMarker(6.1, () -> {
                                    targetSlidePos = RobotConstants.slideAuto;
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide);
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide+RobotConstants.rightSpikeOffset);
                                    robot.intakeMotor.setPower(0);
                                    robot.transferMotor.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(52, -37), Math.toRadians(0))
                .addTemporalMarker(7.3, () -> robot.dropServo.setPosition(RobotConstants.dropPartial))
                .waitSeconds(.3)
                .addTemporalMarker(8.4, () -> targetSlidePos = RobotConstants.slideBottom)
                .lineTo(new Vector2d(45, -37))
                .lineTo(new Vector2d(45, -61))
                .lineTo(new Vector2d(55, -61))
                .build();

        TrajectorySequence left = driveTrain.trajectorySequenceBuilder(new Pose2d(-35, -63, Math.toRadians(90)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(310), 10.62))
                .splineToSplineHeading(new Pose2d(-45, -28, Math.toRadians(0)), Math.toRadians(90))
                .addTemporalMarker(.1, () -> targetSlidePos = RobotConstants.slideBottom)
                .lineTo(new Vector2d(-45, -34))
                .addTemporalMarker(2.3, () -> {robot.leftServo.setPosition(RobotConstants.leftIn);})
                .addTemporalMarker(1.4, () -> {
                                    robot.intakeMotor.setPower(1);
                                    robot.transferMotor.setPower(1);
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack);
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack+RobotConstants.rightSpikeOffset);

                })
                .splineToConstantHeading(new Vector2d(-54, -35), Math.toRadians(180))
                .addTemporalMarker(3.0, () -> {
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn+RobotConstants.rightSpikeOffset);
                })
                .addTemporalMarker(3.3, () -> {
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack);
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack+RobotConstants.rightSpikeOffset);
                })
                .waitSeconds(.5)
                .lineTo(new Vector2d(-53, -35))
                .splineToConstantHeading(new Vector2d(-33, -58), Math.toRadians(0))
                .lineTo(new Vector2d(33, -58))
                .addTemporalMarker(6.1, () -> {
                                    targetSlidePos = RobotConstants.slideAuto;
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide);
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide+RobotConstants.rightSpikeOffset);
                                    robot.intakeMotor.setPower(0);
                                    robot.transferMotor.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(52, -32), Math.toRadians(0))
                .addTemporalMarker(7.6, () -> robot.dropServo.setPosition(RobotConstants.dropPartial))
                .waitSeconds(.3)
                .addTemporalMarker(8.5, () -> targetSlidePos = RobotConstants.slideBottom)
                .lineTo(new Vector2d(45, -32))
                .lineTo(new Vector2d(45, -61))
                .lineTo(new Vector2d(55, -61))
                .build();

        ElapsedTime cameraDelayTimer = new ElapsedTime();

        telemetry.setAutoClear(false);
        Telemetry.Item detectedPos = telemetry.addData("Position", "No detection");
        Telemetry.Item wasPos = telemetry.addData("Was pos", "");

        Telemetry.Item slideData = telemetry.addData("Slide Data:", "Encoder Val:" + robot.liftEncoder.getCurrentPosition() + " Target Val:" + targetSlidePos);


        robot.leftServo.setPosition(RobotConstants.leftOut);
        robot.dropServo.setPosition(RobotConstants.dropClosed);

        waitForStart();
        if (isStopRequested()) return;

        driveTrain.update();
        driveTrain.setPoseEstimate(new Pose2d(-35, -63, Math.toRadians(90)));

        cameraDelayTimer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            driveTrain.update();

            switch (camera) {
                case WAIT:
                    if (cameraDelayTimer.seconds() > 1.5) {
                        camera = fCamRedFarAutoAvoid.Camera.SAVE;
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

                    camera = fCamRedFarAutoAvoid.Camera.FINISHED;
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

        }
    }
}
