package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*public class Auto extends OpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive dt = new SampleMecanumDrive(hardwareMap);

            // in front of trusses on blue
            Trajectory blueFront = dt.trajectoryBuilder(new Pose2d(-35, 63, Math.toRadians(270)))
                    .forward(19)
                    .addDisplacementMarker(() -> {}) // uses camera to look at AprilTags
                    //.waitSeconds(1)
                    //.turn(Math.toRadians(90))
                    .addDisplacementMarker(() -> {}) // places down corresponding pixels
                    //.waitSeconds(1)
                    .splineTo(new Vector2d(-12, 35), Math.toRadians(0))
                    .forward(60)
                    .build();

    }
}*/
