package org.firstinspires.ftc.teamcode.autos.purePursuitAutos;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.RobotConfig;

import java.util.function.DoubleSupplier;

@Autonomous
public class OtherPurePursuitDemo extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry.Item motorPowers = telemetry.addData("Motor powers", "");

        RobotConfig robot = new RobotConfig(hardwareMap);

        Path test = new Path(
                new StartWaypoint(new com.arcrobotics.ftclib.geometry.Pose2d(0, 0, new Rotation2d(0))),
//                new PointTurnWaypoint(new com.arcrobotics.ftclib.geometry.Pose2d(-40, 0, new Rotation2d(Math.toRadians(180))),
//                        DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, 10, .5, Math.toRadians(1)),
                new EndWaypoint(new com.arcrobotics.ftclib.geometry.Pose2d(80, 0, new Rotation2d(Math.toRadians(0))),
                        DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, 30, .3, Math.toRadians(1))
        );



        workaroundodom odom = new workaroundodom(new com.arcrobotics.ftclib.geometry.Pose2d(0, 0, new Rotation2d(0)), robot);

        Motor FL, BL, BR, FR;
        FL = new Motor(hardwareMap, "leftFront");
        BL = new Motor(hardwareMap, "leftRear");
        BR = new Motor(hardwareMap, "rightRear");
        FR = new Motor(hardwareMap, "rightFront");

        FL.setInverted(true);
        BR.setInverted(true);
        BL.setInverted(true);
        FR.setInverted(true);
//        BL.setInverted(true);


//        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
//        backLeft = hardwareMap.get(DcMotorEx.class, "leftRear");
//        backRight = hardwareMap.get(DcMotorEx.class, "rightRear");
//        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");

        com.arcrobotics.ftclib.drivebase.MecanumDrive driveTrain = new com.arcrobotics.ftclib.drivebase.MecanumDrive(FL, FR, BL, BR);

        waitForStart();
//        robot.update();
//        robot.setPoseEstimate();

        test.followPath(driveTrain, odom);

//        robot.followPurePursuitPath(test);
//        while (!isStopRequested()) {
//            // control loop
//            robot.update();
//            motorPowers.setValue(robot.motorPowers());
//            telemetry.update();
//
//        }

    }
}
