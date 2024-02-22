package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.RobotConfig;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin moving forward and
 * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
 * your follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
 * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
 */
@Config
@Autonomous(group = "drive")
public class BackAndForthSpline extends LinearOpMode {

    public static double DISTANCEX = 80;
    public static double DISTANCEY = 25;


    @Override
    public void runOpMode() throws InterruptedException {
        RobotConfig robot = new RobotConfig(hardwareMap);

        TrajectorySequence trajectoryForward = robot.trajectorySequenceBuilder(new Pose2d(-37, 62, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(-51, 35), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-46.5, 34), Math.toRadians(30))
                .waitSeconds(.4)
                .build();

        TrajectorySequence trajectoryBackward = robot.trajectorySequenceBuilder(trajectoryForward.end())
                .splineToSplineHeading(trajectoryForward.start(), Math.toRadians(90))
                .waitSeconds(.3)
                .build();

        waitForStart();

        robot.update();
        robot.setPoseEstimate(new Pose2d(-37, 62, Math.toRadians(270)));

        while (!isStopRequested()) {
            robot.followTrajectorySequence(trajectoryForward);
            robot.followTrajectorySequence(trajectoryBackward);
        }
    }
}