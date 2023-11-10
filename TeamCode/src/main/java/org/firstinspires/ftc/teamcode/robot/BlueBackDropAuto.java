package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.RobotConstants.slidePIDVals;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class BlueBackDropAuto extends LinearOpMode {

    enum SpikeMarkPosition{
        LEFT,
        CENTER,
        RIGHT
    }
    SpikeMarkPosition spikeMarkPosition = SpikeMarkPosition.CENTER;

    int targetPos = RobotConstants.slideBottom;
    double slideI = 0;



    @Override
    public void runOpMode() throws InterruptedException {
        HwMap robot = new HwMap();
        robot.init(hardwareMap);
        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);

        robot.dropServo.setPosition(RobotConstants.dropClosed);

        TrajectorySequence left = driveTrain.trajectorySequenceBuilder(new Pose2d(12, 56, Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(12, 33, Math.toRadians(180)))
                .addTemporalMarker(.2, () -> {targetPos = RobotConstants.slideGround;})
                .lineToSplineHeading(new Pose2d(8, 33, Math.toRadians(180)))
                .addDisplacementMarker(() -> {robot.dropServo.setPosition(RobotConstants.dropOpen);})
                .waitSeconds(.4)
                .addDisplacementMarker(() -> {
                    robot.dropServo.setPosition(RobotConstants.dropClosed);
                    targetPos = RobotConstants.slideLow;
                })
                .forward(-5)
                .splineToSplineHeading(new Pose2d(50, 29, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(() -> {robot.dropServo.setPosition(RobotConstants.dropOpen);})
                .forward(-4)
                .addDisplacementMarker(() -> {
                    robot.dropServo.setPosition(RobotConstants.dropClosed);
                    targetPos = RobotConstants.slideBottom;
                })
                .lineTo(new Vector2d(46,60))
                .build();

        TrajectorySequence center = driveTrain.trajectorySequenceBuilder(new Pose2d(12, 56, Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(12, 33, Math.toRadians(-90)))
                .addTemporalMarker(.2, () -> {targetPos = RobotConstants.slideGround;})
                .addDisplacementMarker(() -> {robot.dropServo.setPosition(RobotConstants.dropOpen);})
                .waitSeconds(.4)
                .addDisplacementMarker(() -> {
                    robot.dropServo.setPosition(RobotConstants.dropClosed);
                    targetPos = RobotConstants.slideLow;
                })
                .lineToSplineHeading(new Pose2d(12, 40, Math.toRadians(0)))
                .lineTo(new Vector2d(12.1, 40))
                .splineTo(new Vector2d(50, 35), Math.toRadians(0))
                .addDisplacementMarker(() -> {robot.dropServo.setPosition(RobotConstants.dropOpen);})
                .waitSeconds(.4)
                .forward(-4)
                .addDisplacementMarker(() -> {
                    robot.dropServo.setPosition(RobotConstants.dropClosed);
                    targetPos = RobotConstants.slideBottom;
                    })
                .lineTo(new Vector2d(46,60))
                .build();

        TrajectorySequence right = driveTrain.trajectorySequenceBuilder(new Pose2d(12, 56, Math.toRadians(-90)))
                .splineToSplineHeading(new Pose2d(25, 35, Math.toRadians(-180)), Math.toRadians(0))
                .addDisplacementMarker(() -> {robot.dropServo.setPosition(RobotConstants.dropOpen);})
                .forward(-5)
                .splineToSplineHeading(new Pose2d(50, 40, Math.toRadians(0)), Math.toRadians(0))
                .forward(-4)
                .lineTo(new Vector2d(46,60))
                .build();

        waitForStart();
        switch (spikeMarkPosition) {
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
        while (opModeIsActive() && !isStopRequested()) {
            driveTrain.update();


            double slideVelo = robot.liftEncoder.getCorrectedVelocity();
            int slideCurPos = robot.liftEncoder.getCurrentPosition();

            double distRemain = targetPos - slideCurPos;

            slideI += distRemain * slidePIDVals.i;

            double slidePower = (distRemain * slidePIDVals.p) + slideI + (slideVelo * slidePIDVals.d);

            robot.liftMotor.setPower(slidePower);
        }


    }



}
