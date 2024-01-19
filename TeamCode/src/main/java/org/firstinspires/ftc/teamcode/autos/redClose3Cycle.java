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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class redClose3Cycle {
        enum Camera {
            WAIT,
            SAVE,
            FINISHED
        }
        org.firstinspires.ftc.teamcode.autos.fCamRedCloseAuto.Camera camera = org.firstinspires.ftc.teamcode.autos.fCamRedCloseAuto.Camera.WAIT;

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
                    pipeline.setAlliance("Red");
                    robot.webcam.setPipeline(pipeline);

                    robot.webcam.startStreaming(640,480, OpenCvCameraRotation.UPSIDE_DOWN);
                }

                @Override
                public void onError(int errorCode) {
                    telemetry.addData("Error: ", errorCode);
                }
            });
            enum Intake {
                WAIT,
                INTAKE_DEPLOY,
                INTAKE_DEPLOY_ENDING_SEQUENCE,
                START_SPIN,
                PIXEL_A,
                PIXEL_B,
                TRANSFER_DELAY
            }

            Intake intake = Intake.WAIT;

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
                    .waitSeconds(.3)
                    .lineTo(new Vector2d(40, -28.5))
                    .addDisplacementMarker(() -> {targetSlidePos = RobotConstants.slideBottom; robot.dropServo.setPosition(RobotConstants.dropClosed);})
                    //.lineToConstantHeading(new Vector2d(-60, -11))
                TrajectorySequence leftToStack = driveTrain.trajectoryBuilder(left.end())
                    .splineToConstantHeading(new Vector2d(17, -6.6), Math.toRadians(180))
                    .lineTo(new Vector2d(-34, -6.6))
                    .splineToConstantHeading(new Vector2d(-60,-11), Math.toRadians(180))
                    //fix temporal marker and intake
                    .addTemporalMarker(1.2, () -> intake = Intake.INTAKE_DEPLOY)
                    //.waitSeconds(0.3)
                    .splineToConstantHeading(new Vector2d(-34,-6.6), Math.toRadians(0))
                    .lineTo(new Vector2d(19, -6.6))
                    .splineToConstantHeading(new Vector2d(54.3, -37.5), Math.toRadians(0))
                    .build();

            TrajectorySequence center = driveTrain.trajectorySequenceBuilder(new Pose2d(12, -63, Math.toRadians(90)))
                    .lineTo(new Vector2d(12, -60))
                    .splineToSplineHeading(new Pose2d(17, -30), Math.toRadians(90))
                    .lineTo(new Vector2d(17,-36))
                    .addTemporalMarker(2.0, () -> robot.leftServo.setPosition(RobotConstants.leftIn))
                    .lineTo(new Vector2d(18, -36))
                    .splineToConstantHeading(new Vector2d(45, -37.5), Math.toRadians(0))
                    .lineTo(new Vector2d(54.2, -37.5))
                    .lineTo(new Vector2d(54.3, -37.5))
                    .addTemporalMarker(3.9, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
                    .waitSeconds(.3)
                    .build();

            TrajectorySequence centerToStack = driveTrain.trajectorySequenceBuilder(center.end())
                    .lineToConstantHeading(new Vector2d(54.1,-37.5))
                    .splineToConstantHeading(new Vector2d(19, -6.6), Math.toRadians(180))
                    .lineTo(new Vector2d(-34, -6.6))
                    .splineToConstantHeading(new Vector2d(-60,-11), Math.toRadians(180))
                    .addTemporalMarker(1, () -> targetSlidePos = RobotConstants.slideBottom)
                    //will have to fix this intake enum
                    //fix temporalMarker timings
                    //.waitSeconds(0.3)
                    .addTemporalMarker(1.2, () -> intake = Intake.INTAKE_DEPLOY)
                    .splineToConstantHeading(new Vector2d(-34,-6.6), Math.toRadians(0))
                    .lineTo(new Vector2d(19, -6.6))
                    .splineToConstantHeading(new Vector2d(54.3, -37.5), Math.toRadians(0))

                    .build();

            /*TrajectorySequence right = driveTrain.trajectorySequenceBuilder(new Pose2d(12, -63, Math.toRadians(90)))
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
                    .lineTo(new Vector2d(40, -45))
                    .addDisplacementMarker(() -> {targetSlidePos = RobotConstants.slideBottom; robot.dropServo.setPosition(RobotConstants.dropClosed);})
                    .lineTo(new Vector2d(40, -60))
                    .lineTo(new Vector2d(45, -60))
                    .build();*/



            ElapsedTime cameraDelayTimer = new ElapsedTime();

            telemetry.setAutoClear(false);
            Telemetry.Item detectedPos = telemetry.addData("Position", "No detection");

            Telemetry.Item slideData = telemetry.addData("Slide Data:", "Encoder Val:" + robot.liftEncoder.getCurrentPosition() + " Target Val:" + targetSlidePos);

            robot.leftServo.setPosition(RobotConstants.leftOut);
            robot.dropServo.setPosition(RobotConstants.dropClosed);

            waitForStart();
            if (isStopRequested()) return;

            robot.climbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            driveTrain.setPoseEstimate(new Pose2d(12, -63, Math.toRadians(90)));

            cameraDelayTimer.reset();

            while (opModeIsActive() && !isStopRequested()) {
                driveTrain.update();

                switch (camera) {
                    case WAIT:
                        if (cameraDelayTimer.seconds() > 1.5) {
                            camera = org.firstinspires.ftc.teamcode.autos.fCamRedCloseAuto.Camera.SAVE;
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
                        camera = org.firstinspires.ftc.teamcode.autos.fCamRedCloseAuto.Camera.FINISHED;
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

            }

        }
    }

}
