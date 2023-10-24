package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Auto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive dt = new SampleMecanumDrive(hardwareMap);

        Pose2d bFStartPose = new Pose2d(-35, 63, Math.toRadians(270));
        Pose2d bBStartPose = new Pose2d(12, 63, Math.toRadians(270));
        Pose2d rFStartPose = new Pose2d(-35, -63, Math.toRadians(90));
        Pose2d rBStartPose = new Pose2d(12, -63, Math.toRadians(90));


        // in front of trusses on blue

        dt.setPoseEstimate(bFStartPose);
        TrajectorySequence blueFront = dt.trajectorySequenceBuilder(bFStartPose)
                .forward(19)
                .addDisplacementMarker(() -> { // uses Vision to detect where the team prop is

                })
                .waitSeconds(1)
                .addDisplacementMarker(( // places down pixel where team prop is

                ) -> {})
                .waitSeconds(1)
                .splineTo(new Vector2d(-12, 35), Math.toRadians(0))
                .forward(60)
                .addDisplacementMarker(() -> { // places down pixel on backdrop

                })
                // cycling
                .waitSeconds(1)
                .back(10)
                .turn(Math.toRadians(180))
                .forward(90)
                .addDisplacementMarker(() -> {// picks up pixels

                })
                .waitSeconds(1)
                .back(10)
                .turn(Math.toRadians(180))
                .forward(90)
                .addDisplacementMarker(() -> { // place down pixel on backdrop

                })

                .build();

        waitForStart();

        if (!isStopRequested()) {
            dt.followTrajectorySequence(blueFront);
        }
    }
}
