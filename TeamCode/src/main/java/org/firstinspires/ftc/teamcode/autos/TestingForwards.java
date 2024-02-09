package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.HwMap;

@Autonomous
public class TestingForwards extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);
        HwMap robot = new HwMap();
        robot.init(hardwareMap);


        Trajectory forwards = driveTrain.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .forward(70)
                .build();

        waitForStart();
        driveTrain.update();
        driveTrain.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        driveTrain.followTrajectory(forwards);
    }
}
