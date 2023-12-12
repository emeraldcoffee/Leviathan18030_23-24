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

@Autonomous(name = "fCam Blue Far Auto Avoid")
public class fCamBlueFarAutoAvoid extends LinearOpMode {

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

    double slideI = 0;

    String pos = "";

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);
        ColorMask pipeline = new ColorMask();
        HwMap robot = new HwMap();
        robot.init(hardwareMap);

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

        TrajectorySequence left = driveTrain.trajectorySequenceBuilder(new Pose2d(-35, 63, Math.toRadians(270)))
                .addDisplacementMarker(() -> targetSlidePos = RobotConstants.slideBottom)
                .lineTo(new Vector2d(-35, 60))
                .splineToConstantHeading(new Vector2d(-31, 34), Math.toRadians(280))
                .addSpatialMarker(new Vector2d(-31, 34), () -> robot.leftServo.setPosition(RobotConstants.leftIn))
                .waitSeconds(.2)
                .lineTo(new Vector2d(-36, 32))
                .splineToSplineHeading(new Pose2d(-40, 20, Math.toRadians(0)), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-32, 14), Math.toRadians(0))
                .lineTo(new Vector2d(-12, 14))
                .waitSeconds(13)
                .lineTo(new Vector2d(10, 14))
                .splineToConstantHeading(new Vector2d(45, 44), Math.toRadians(0))
                .addSpatialMarker(new Vector2d(25, 22), () -> targetSlidePos = RobotConstants.slideAuto)
                .lineTo(new Vector2d(54.5, 44))
                .lineTo(new Vector2d(54.6, 44))
                .addTemporalMarker(6.8 +13, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
                .waitSeconds(.6)
                .lineTo(new Vector2d(40, 44))
                .addDisplacementMarker(() -> {targetSlidePos = RobotConstants.slideBottom; robot.dropServo.setPosition(RobotConstants.dropClosed);})
                .lineTo(new Vector2d(40, 10))
                .lineTo(new Vector2d(45, 10))
                .build();

        TrajectorySequence center = driveTrain.trajectorySequenceBuilder(new Pose2d(-35, 63, Math.toRadians(270)))
                .addDisplacementMarker(() -> targetSlidePos = RobotConstants.slideBottom)
                .lineTo(new Vector2d(-35, 60))
                .splineToSplineHeading(new Pose2d(-34, 17, Math.toRadians(0)), Math.toRadians(280))
                .addSpatialMarker(new Vector2d(-34, 17), () -> robot.leftServo.setPosition(RobotConstants.leftIn))
                .waitSeconds(.2)
                .lineTo(new Vector2d(-34, 16.5))
                .splineToConstantHeading(new Vector2d(-12, 13), Math.toRadians(0))
                .waitSeconds(13)
                .lineTo(new Vector2d(10, 13))
                .splineToConstantHeading(new Vector2d(45, 32), Math.toRadians(0))
                .addSpatialMarker(new Vector2d(26, 22), () -> targetSlidePos = RobotConstants.slideAuto)
                .lineTo(new Vector2d(55.5, 32))
                .lineTo(new Vector2d(55.6, 32))
                .addTemporalMarker(6.3+13, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
                .waitSeconds(.6)
                .lineTo(new Vector2d(40, 32))
                .addDisplacementMarker(() -> {targetSlidePos = RobotConstants.slideBottom; robot.dropServo.setPosition(RobotConstants.dropClosed);})
                .lineTo(new Vector2d(40, 10))
                .lineTo(new Vector2d(45, 10))
                .build();

        TrajectorySequence right = driveTrain.trajectorySequenceBuilder(new Pose2d(-35, 63, Math.toRadians(270)))
                .addDisplacementMarker(() -> targetSlidePos = RobotConstants.slideBottom)
                .lineTo(new Vector2d(-35, 60))
                .splineToSplineHeading(new Pose2d(-45, 21, Math.toRadians(0)), Math.toRadians(280))
                .addSpatialMarker(new Vector2d(-45, 21), () -> robot.leftServo.setPosition(RobotConstants.leftIn))
                .waitSeconds(.2)
                .lineTo(new Vector2d(-42, 19))
                .splineToConstantHeading(new Vector2d(-30, 14), Math.toRadians(0))
                .lineTo(new Vector2d(-12, 14))
                .waitSeconds(13)
                .lineTo(new Vector2d(10, 14))
                .splineToConstantHeading(new Vector2d(45, 23), Math.toRadians(0))
                .addSpatialMarker(new Vector2d(38, 36), () -> targetSlidePos = RobotConstants.slideAuto)
                .lineTo(new Vector2d(54.5, 23))
                .lineTo(new Vector2d(54.6, 23))
                .addTemporalMarker(6.3+13, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
                .waitSeconds(.6)
                .lineTo(new Vector2d(40, 23))
                .addDisplacementMarker(() -> {targetSlidePos = RobotConstants.slideBottom; robot.dropServo.setPosition(RobotConstants.dropClosed);})
                .lineTo(new Vector2d(40, 10))
                .lineTo(new Vector2d(45, 10))

                .build();

        ElapsedTime cameraDelayTimer = new ElapsedTime();

        telemetry.setAutoClear(false);
        Telemetry.Item detectedPos = telemetry.addData("Position", "No detection");

        Telemetry.Item slideData = telemetry.addData("Slide Data:", "Encoder Val:" + robot.liftEncoder.getCurrentPosition() + " Target Val:" + targetSlidePos);



        robot.leftServo.setPosition(RobotConstants.leftOut);
        robot.dropServo.setPosition(RobotConstants.dropClosed);

        waitForStart();
        if (isStopRequested()) return;

        robot.climbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveTrain.setPoseEstimate(new Pose2d(-35, 63, Math.toRadians(270)));

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
