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

    SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);
    HwMap robot = new HwMap();

    TrajectorySequence left = driveTrain.trajectorySequenceBuilder(new Pose2d(12, -63, Math.toRadians(90)))
            .lineTo(new Vector2d(12, -45))
            .splineToConstantHeading(new Vector2d(10, -33), Math.toRadians(90))
            .addSpatialMarker(new Vector2d(45, -29), () -> robot.leftServo.setPosition(RobotConstants.leftIn))
            .waitSeconds(.2)
            .lineTo(new Vector2d(20, -33))
            .splineToSplineHeading(new Pose2d(45, -29, Math.toRadians(0)), Math.toRadians(0))
            .addSpatialMarker(new Vector2d(38, -29), () -> targetSlidePos = RobotConstants.slideLow)
            .lineTo(new Vector2d(54.5, -29))
            .lineTo(new Vector2d(54.6, -29))
            .addSpatialMarker(new Vector2d(54.6, -29), () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
            .waitSeconds(.3)
            .lineTo(new Vector2d(40, -29))
            .addDisplacementMarker(() -> {targetSlidePos = RobotConstants.slideBottom; robot.dropServo.setPosition(RobotConstants.dropClosed);})
            .lineTo(new Vector2d(40, -60))
            .lineTo(new Vector2d(45, -60))
            .build();

    TrajectorySequence center = driveTrain.trajectorySequenceBuilder(new Pose2d(12, -63, Math.toRadians(90)))
            .lineTo(new Vector2d(12, -60))
            .splineToSplineHeading(new Pose2d(18, -34), Math.toRadians(90))
            .addSpatialMarker(new Vector2d(18, -34), () -> robot.leftServo.setPosition(RobotConstants.leftIn))
            .waitSeconds(.2)
            .lineTo(new Vector2d(19, 34))
            .splineToConstantHeading(new Vector2d(45, -39), Math.toRadians(0))
            .addSpatialMarker(new Vector2d(38, -39), () -> targetSlidePos = RobotConstants.slideLow)
            .lineTo(new Vector2d(54.5, -39))
            .lineTo(new Vector2d(54.6, -39))
            .addSpatialMarker(new Vector2d(54.6, -39), () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
            .waitSeconds(.3)
            .lineTo(new Vector2d(40, -39))
            .addDisplacementMarker(() -> targetSlidePos = RobotConstants.slideBottom)
            .lineTo(new Vector2d(40, -60))
            .lineTo(new Vector2d(45, -60))
            .build();

    TrajectorySequence right = driveTrain.trajectorySequenceBuilder(new Pose2d(12, -63, Math.toRadians(90)))
            .lineTo(new Vector2d(12, -61))
            .lineTo(new Vector2d(13, -59))
            .splineToSplineHeading(new Pose2d(26, -37), Math.toRadians(90))
            .addSpatialMarker(new Vector2d(26, -37), () -> robot.leftServo.setPosition(RobotConstants.leftIn))
            .waitSeconds(.2)
            .lineTo(new Vector2d(27, -37))
            .splineToConstantHeading(new Vector2d(45, -43), Math.toRadians(0))
            .addSpatialMarker(new Vector2d(38, -39), () -> targetSlidePos = RobotConstants.slideLow)
            .lineTo(new Vector2d(54.5, -43))
            .lineTo(new Vector2d(54.6, -43))
            .addSpatialMarker(new Vector2d(54.6, -39), () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
            .waitSeconds(.3)
            .lineTo(new Vector2d(40, -43))
            .addDisplacementMarker(() -> targetSlidePos = RobotConstants.slideBottom)
            .lineTo(new Vector2d(40, -60))
            .lineTo(new Vector2d(45, -60))
            .build();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        ElapsedTime timer = new ElapsedTime();

        Telemetry.Item slideData = telemetry.addData("Slide Data:", "Encoder Val:" + robot.liftEncoder.getCurrentPosition() + " Target Val:" + targetSlidePos);


        robot.leftServo.setPosition(RobotConstants.leftOut);
        robot.dropServo.setPosition(RobotConstants.dropClosed);

        waitForStart();

        driveTrain.setPoseEstimate(new Pose2d(12, -63, Math.toRadians(90)));

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
