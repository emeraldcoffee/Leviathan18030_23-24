package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.HwMap;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class TestingBack extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);
        HwMap robot = new HwMap();
        robot.init(hardwareMap);


        Trajectory backwards = driveTrain.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .back(70)
                .build();

        TrajectorySequence forwards = driveTrain.trajectorySequenceBuilder(new Pose2d(13.4, 64, Math.toRadians(270)))
                .forward(70)
                .build();

        waitForStart();
        driveTrain.update();
        driveTrain.setPoseEstimate(new Pose2d(13.4, 64, Math.toRadians(270)));

        driveTrain.followTrajectory(backwards);
    }
}
