package org.firstinspires.ftc.teamcode.autos.old;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.RobotConfig;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.HwMap;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous
public class TestingForwards extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotConfig robot = new RobotConfig(hardwareMap);
//        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);
//        HwMap robot = new HwMap();
//        robot.init(hardwareMap);


        Trajectory backwards = robot.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .back(70)
                .build();

        Trajectory forwards = robot.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .forward(70)
                .build();

        waitForStart();
        robot.update();
        robot.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        robot.followTrajectory(forwards);
    }
}
