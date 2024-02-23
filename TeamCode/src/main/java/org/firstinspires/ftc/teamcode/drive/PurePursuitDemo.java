package org.firstinspires.ftc.teamcode.drive;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.PointTurnWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Disabled
@Autonomous
public class PurePursuitDemo extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        RobotConfig robot = new RobotConfig(hardwareMap);
        robot.setPoseEstimate(new Pose2d(53, 27, Math.toRadians(0)));

        Path test = new Path(
                new StartWaypoint(53, 27),
                new PointTurnWaypoint(new com.arcrobotics.ftclib.geometry.Pose2d(53, 0, new Rotation2d(Math.toRadians(180))),
                        DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, 10, .5, Math.toRadians(1)),
                new EndWaypoint(new com.arcrobotics.ftclib.geometry.Pose2d(-40, 0, new Rotation2d(Math.toRadians(180))),
                        DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, 10, .3, Math.toRadians(1))
        );


        waitForStart();

        robot.followPurePursuitPath(test);
        while (!isStopRequested()) {
            // control loop
            robot.update();

        }

    }
}
