package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "fCam Red Close Auto")
public class fCamRedCloseAuto extends LinearOpMode {

    enum AutoPath {
        LEFT,
        CENTER,
        RIGHT
    }
    AutoPath autoPath = AutoPath.RIGHT;

    int targetSlidePos = RobotConstants.slideBottom;

    double slideI = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);
        HwMap robot = new HwMap();
        robot.init(hardwareMap);

        ElapsedTime timer = new ElapsedTime();

        Telemetry.Item slideData = telemetry.addData("Slide Data:", "Encoder Val:" + robot.liftEncoder.getCurrentPosition() + " Target Val:" + targetSlidePos);

        driveTrain.setPoseEstimate(new Pose2d(12, -63, Math.toRadians(90)));

        TrajectorySequence left = driveTrain.trajectorySequenceBuilder(new Pose2d(12, -63, Math.toRadians(90)))
                .lineTo(new Vector2d(12, -45))
                .splineToConstantHeading(new Vector2d(10, -33), Math.toRadians(90))
                .addDisplacementMarker(() -> robot.leftServo.setPosition(RobotConstants.leftIn))
                .lineTo(new Vector2d(20, -33))
                .splineToSplineHeading(new Pose2d(45, -28, Math.toRadians(0)), Math.toRadians(0))
                .addSpatialMarker(new Vector2d(38, -28), () -> targetSlidePos = RobotConstants.slideLow)
                .lineTo(new Vector2d(50, -28))
                .addDisplacementMarker(() -> {})
                .lineTo(new Vector2d(40, -28))
                .addDisplacementMarker(() -> targetSlidePos = RobotConstants.slideBottom)
                .lineTo(new Vector2d(40, -60))
                .lineTo(new Vector2d(43, -60))
                .build();

        TrajectorySequence center = driveTrain.trajectorySequenceBuilder(new Pose2d(12, -63, Math.toRadians(90)))
                .lineTo(new Vector2d(12, -60))
                .splineToSplineHeading(new Pose2d(15, -34), Math.toRadians(90))
                .addDisplacementMarker(() -> robot.leftServo.setPosition(RobotConstants.leftIn))
                .lineTo(new Vector2d(16, -34))
                .splineToConstantHeading(new Vector2d(45, -34), Math.toRadians(0))
                .addSpatialMarker(new Vector2d(38, -23), () -> targetSlidePos = RobotConstants.slideLow)
                .lineTo(new Vector2d(50, -34))
                .addDisplacementMarker(() -> {})
                .lineTo(new Vector2d(40, -34))
                .addDisplacementMarker(() -> targetSlidePos = RobotConstants.slideBottom)
                .lineTo(new Vector2d(40, -60))
                .lineTo(new Vector2d(43, -60))
                .build();

        TrajectorySequence right = driveTrain.trajectorySequenceBuilder(new Pose2d(12, -63, Math.toRadians(90)))
                .lineTo(new Vector2d(12, -60))
                .splineToSplineHeading(new Pose2d(23, -40), Math.toRadians(0))
                .addDisplacementMarker(() -> robot.leftServo.setPosition(RobotConstants.leftIn))
                .lineTo(new Vector2d(24, -40))
                .splineToConstantHeading(new Vector2d(45, -45), Math.toRadians(0))
                .addSpatialMarker(new Vector2d(38, -45), () -> targetSlidePos = RobotConstants.slideLow)
                .lineTo(new Vector2d(50, -45))
                .addDisplacementMarker(() -> {})
                .lineTo(new Vector2d(40, -45))
                .addDisplacementMarker(() -> targetSlidePos = RobotConstants.slideBottom)
                .lineTo(new Vector2d(40, -60))
                .lineTo(new Vector2d(43, -60))
                .build();

        robot.rightServo.setPosition(RobotConstants.rightOut);

        waitForStart();

        switch (autoPath) {
            case LEFT:
                driveTrain.followTrajectorySequenceAsync(left);
                break;
            case CENTER:
                driveTrain.followTrajectorySequenceAsync(center);
                break;
            case RIGHT:
                driveTrain.followTrajectorySequenceAsync(right);
                break;
        }


        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            driveTrain.update();


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
