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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "fCam Red Far Auto Place Only")
public class fCamRedFarAutoPlaceOnly extends LinearOpMode {

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
//    AutoPath autoPath = AutoPath.LEFT;

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

        TrajectorySequence right = driveTrain.trajectorySequenceBuilder(new Pose2d(-35, -63, Math.toRadians(90)))
                .addDisplacementMarker(() -> targetSlidePos = RobotConstants.slideBottom)
                .lineTo(new Vector2d(-35, -40))
                .splineToConstantHeading(new Vector2d(-31, -34), Math.toRadians(100))
                .addSpatialMarker(new Vector2d(-31, -34), () -> robot.rightServo.setPosition(RobotConstants.rightIn))
                .waitSeconds(.2)
                .lineTo(new Vector2d(-38, -33))
                .splineToSplineHeading(new Pose2d(-41, -20, Math.toRadians(0)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-32, -10.5), Math.toRadians(0))
                .lineTo(new Vector2d(-12, -10.5))
//                .waitSeconds(15)
//                .lineTo(new Vector2d(10, -10.5))
//                .splineToConstantHeading(new Vector2d(45, -54), Math.toRadians(0))
//                .addSpatialMarker(new Vector2d(25, -22), () -> targetSlidePos = RobotConstants.slideAuto)
//                .lineTo(new Vector2d(54.5, -54))
//                .lineTo(new Vector2d(54.6, -54))
//                .addTemporalMarker(7.2+15, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
//                .waitSeconds(.6)
//                .lineTo(new Vector2d(40, -54))
//                .addDisplacementMarker(() -> {targetSlidePos = RobotConstants.slideBottom; robot.dropServo.setPosition(RobotConstants.dropClosed);})
//                .lineTo(new Vector2d(40, -16))
//                .lineTo(new Vector2d(50, -16))
                .build();

        TrajectorySequence center = driveTrain.trajectorySequenceBuilder(new Pose2d(-35, -63, Math.toRadians(90)))
                .addDisplacementMarker(() -> targetSlidePos = RobotConstants.slideBottom)
                .lineTo(new Vector2d(-35, -60))
                .splineToSplineHeading(new Pose2d(-36, -17, Math.toRadians(0)), Math.toRadians(100))
                .addSpatialMarker(new Vector2d(-36, -17), () -> robot.rightServo.setPosition(RobotConstants.rightIn))
                .waitSeconds(.2)
                .lineTo(new Vector2d(-36, -16.5))
                .splineToConstantHeading(new Vector2d(-12, -12), Math.toRadians(0))
//                .waitSeconds(15)
//                .lineTo(new Vector2d(10, -12))
//                .splineToConstantHeading(new Vector2d(45, -42), Math.toRadians(0))
//                .addSpatialMarker(new Vector2d(26, -22), () -> targetSlidePos = RobotConstants.slideAuto)
//                .lineTo(new Vector2d(55.5, -42))
//                .lineTo(new Vector2d(55.6, -42))
//                .addTemporalMarker(6.5+15, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
//                .waitSeconds(.6)
//                .lineTo(new Vector2d(40, -42))
//                .addDisplacementMarker(() -> {targetSlidePos = RobotConstants.slideBottom; robot.dropServo.setPosition(RobotConstants.dropClosed);})
//                .lineTo(new Vector2d(40, -16))
//                .lineTo(new Vector2d(50, -16))
                .build();

        TrajectorySequence left = driveTrain.trajectorySequenceBuilder(new Pose2d(-35, -63, Math.toRadians(90)))
                .addDisplacementMarker(() -> targetSlidePos = RobotConstants.slideBottom)
                .lineTo(new Vector2d(-35, -60))
                .splineToSplineHeading(new Pose2d(-45, -19, Math.toRadians(0)), Math.toRadians(100))
                .addSpatialMarker(new Vector2d(-45, -19), () -> robot.rightServo.setPosition(RobotConstants.rightIn))
                .waitSeconds(.2)
                .lineTo(new Vector2d(-44, -17))
                .splineToConstantHeading(new Vector2d(-30, -11), Math.toRadians(0))
                .lineTo(new Vector2d(-12, -11))
//                .waitSeconds(15)
//                .lineTo(new Vector2d(10, -11))
//                .splineToConstantHeading(new Vector2d(45, -34.5), Math.toRadians(0))
//                .addSpatialMarker(new Vector2d(38, -36), () -> targetSlidePos = RobotConstants.slideAuto)
//                .lineTo(new Vector2d(54.5, -34.5))
//                .lineTo(new Vector2d(54.6, -34.5))
//                .addTemporalMarker(6.5+15, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
//                .waitSeconds(.6)
//                .lineTo(new Vector2d(40, -34.5))
//                .addDisplacementMarker(() -> {targetSlidePos = RobotConstants.slideBottom; robot.dropServo.setPosition(RobotConstants.dropClosed);})
//                .lineTo(new Vector2d(40, -10))
//                .lineTo(new Vector2d(45, -10))

                .build();

        ElapsedTime cameraDelayTimer = new ElapsedTime();

        telemetry.setAutoClear(false);
        Telemetry.Item detectedPos = telemetry.addData("Position", "No detection");

        Telemetry.Item slideData = telemetry.addData("Slide Data:", "Encoder Val:" + robot.liftEncoder.getCurrentPosition() + " Target Val:" + targetSlidePos);



        robot.rightServo.setPosition(RobotConstants.rightOut);
        robot.dropServo.setPosition(RobotConstants.dropClosed);

        waitForStart();
        if (isStopRequested()) return;

        robot.climbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveTrain.setPoseEstimate(new Pose2d(-35, -63, Math.toRadians(90)));

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

        }

    }
}
