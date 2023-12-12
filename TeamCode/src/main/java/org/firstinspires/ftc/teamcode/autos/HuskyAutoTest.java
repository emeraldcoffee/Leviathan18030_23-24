package org.firstinspires.ftc.teamcode.autos;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.ColorMask;
import org.firstinspires.ftc.teamcode.robot.HwMap;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.huskyLensDetection;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "HuskyAutoTest")
public class HuskyAutoTest extends LinearOpMode {
    int targetSlidePos = RobotConstants.slideAuto;

    double slideI = 0;

    String pos = "";
    huskyLensDetection h = new huskyLensDetection();

    enum Camera {
        WAIT,
        SAVE,
        FINISHED
    }

    fCamRedCloseAuto.Camera camera = fCamRedCloseAuto.Camera.WAIT;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);
        ColorMask pipeline = new ColorMask();
        HwMap robot = new HwMap();
        robot.init(hardwareMap);

        HuskyLens huskylens = robot.huskylens;

        pos = h.getPos(huskylens);


        TrajectorySequence left = driveTrain.trajectorySequenceBuilder(new Pose2d(12, -63, Math.toRadians(90)))
                .lineTo(new Vector2d(-35, 47))
                .splineToConstantHeading(new Vector2d(-20,35), Math.toRadians(0))
                .addSpatialMarker(new Vector2d(8.5, -36), () -> robot.leftServo.setPosition(RobotConstants.leftIn))
                .waitSeconds(.2)
                .lineTo(new Vector2d(-28, 35))
                .splineToConstantHeading(new Vector2d(-36, 48), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-20, 60), Math.toRadians(0))
                .lineTo(new Vector2d(13, 60))
                .splineToConstantHeading(new Vector2d(54.6, 42), Math.toRadians(0))
                .waitSeconds(.2)

                .build();

        TrajectorySequence center = driveTrain.trajectorySequenceBuilder(new Pose2d(12, -63, Math.toRadians(90)))
                .lineTo(new Vector2d(12, -60))
                .splineToSplineHeading(new Pose2d(17, -30), Math.toRadians(90))
                .lineTo(new Vector2d(17, -36))
                .addTemporalMarker(2.2, () -> robot.leftServo.setPosition(RobotConstants.leftIn))
//                                .addSpatialMarker(new Vector2d(15, 35), () -> robot.rightServo.setPosition(RobotConstants.rightIn))
                .waitSeconds(.2)
                .lineTo(new Vector2d(18, -36))
                .splineToConstantHeading(new Vector2d(45, -37.5), Math.toRadians(0))
                .lineTo(new Vector2d(54.5, -37.5))
                .lineTo(new Vector2d(54.6, -37.5))
                .addSpatialMarker(new Vector2d(54.6, -37.5), () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
                .waitSeconds(.2)
                .lineTo(new Vector2d(40, -37.5))
                .addDisplacementMarker(() -> {
                    targetSlidePos = RobotConstants.slideBottom;
                    robot.dropServo.setPosition(RobotConstants.dropClosed);
                })
                .lineTo(new Vector2d(40, -60))
                .lineTo(new Vector2d(45, -60))
                .build();

        TrajectorySequence right = driveTrain.trajectorySequenceBuilder(new Pose2d(12, -63, Math.toRadians(90)))
                .lineTo(new Vector2d(12, -61))
                .splineToLinearHeading(new Pose2d(25, -45), Math.toRadians(90))
                .addSpatialMarker(new Vector2d(25, -45), () -> robot.leftServo.setPosition(RobotConstants.leftIn))
                .waitSeconds(.2)
                .lineTo(new Vector2d(25, -46))
                .lineTo(new Vector2d(26, -46))
                .splineToConstantHeading(new Vector2d(45, -46), Math.toRadians(0))
                .lineTo(new Vector2d(54.5, -46))
                .lineTo(new Vector2d(54.6, -46))
                .addSpatialMarker(new Vector2d(54.6, -46), () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
                .waitSeconds(.2)
                .lineTo(new Vector2d(40, -46))
                .addDisplacementMarker(() -> {
                    targetSlidePos = RobotConstants.slideBottom;
                    robot.dropServo.setPosition(RobotConstants.dropClosed);
                })
                .lineTo(new Vector2d(40, -60))
                .lineTo(new Vector2d(45, -60))
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

        driveTrain.setPoseEstimate(new Pose2d(12, -63, Math.toRadians(90)));

        cameraDelayTimer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            driveTrain.update();

            switch (camera) {
                case WAIT:
                    if (cameraDelayTimer.seconds() > 1.5) {
                        camera = fCamRedCloseAuto.Camera.SAVE;
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
                    camera = fCamRedCloseAuto.Camera.FINISHED;
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
