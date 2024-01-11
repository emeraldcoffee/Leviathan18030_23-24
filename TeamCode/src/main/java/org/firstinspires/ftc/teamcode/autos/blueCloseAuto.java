package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.HwMap;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Disabled
@Autonomous(name = "Blue Close Auto")
public class blueCloseAuto extends LinearOpMode {

    enum AutoPath {
        LEFT,
        CENTER,
        RIGHT
    }
    AutoPath autoPath = AutoPath.RIGHT;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);
        HwMap robot = new HwMap();
        robot.init(hardwareMap);

        ElapsedTime timer = new ElapsedTime();

        driveTrain.setPoseEstimate(new Pose2d(12, 63, Math.toRadians(0)));

        TrajectorySequence left = driveTrain.trajectorySequenceBuilder(new Pose2d(12, 63, Math.toRadians(0)))
                .lineTo(new Vector2d(12, 62))
                .splineToLinearHeading(new Pose2d(25, 40, Math.toRadians(0)), Math.toRadians(270))
                .addDisplacementMarker(() -> {
                    robot.rightServo.setPosition(RobotConstants.rightIn);
                })
                .addSpatialMarker(new Vector2d(27, 40), () -> {

                })
                .lineTo(new Vector2d(29, 40))
                .splineToLinearHeading(new Pose2d(50, 40, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(() -> {})
                .lineTo(new Vector2d(43, 40))
                .addDisplacementMarker(() -> {

                })
                .lineTo(new Vector2d(43, 60))
                .build();

        TrajectorySequence center = driveTrain.trajectorySequenceBuilder(new Pose2d(12, 63, Math.toRadians(0)))
                                .lineTo(new Vector2d(12, 62))
                .splineToLinearHeading(new Pose2d(14, 34, Math.toRadians(0)), Math.toRadians(270))
                .addDisplacementMarker(() -> {
                    robot.rightServo.setPosition(RobotConstants.rightIn);
                })
                .addSpatialMarker(new Vector2d(16, 34), () -> {

                })
                .lineTo(new Vector2d(16, 34))
                .splineToLinearHeading(new Pose2d(50, 36, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(() -> {})
                .lineTo(new Vector2d(43, 36))
                .addDisplacementMarker(() -> {

                })
                .lineTo(new Vector2d(43, 60))
                .build();

        TrajectorySequence right = driveTrain.trajectorySequenceBuilder(new Pose2d(12, 63, Math.toRadians(0)))
                                .lineTo(new Vector2d(12.2, 62))
                .splineToSplineHeading(new Pose2d(14, 35, Math.toRadians(270)), Math.toRadians(270))
                .lineTo(new Vector2d(11, 35))
                .addDisplacementMarker(() -> {
                    robot.rightServo.setPosition(RobotConstants.rightIn);
                })
                .lineTo(new Vector2d(15, 35))
                .splineToSplineHeading(new Pose2d(45, 30, Math.toRadians(0)), Math.toRadians(0))
                .addSpatialMarker(new Vector2d(25, 34), () -> {

                })
                .lineTo(new Vector2d(50, 30))
                .addDisplacementMarker(() -> {

                })
                .lineTo(new Vector2d(45, 30))

                .lineTo(new Vector2d(43, 30))
                .addDisplacementMarker(() -> {

                })
                .lineTo(new Vector2d(43, 60))
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
        }

    }
}
