package org.firstinspires.ftc.teamcode.drive;


import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PurePursuitDemo extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        RobotConfig robot = new RobotConfig(hardwareMap);

        PurePursuitCommand forward = new PurePursuitCommand(robot.driveTrain, robot.odometry,
                new StartWaypoint(0, 0),
                new GeneralWaypoint(200, 0, 0.8, 0.8, 30),
                new EndWaypoint(
                        400, 0, 0, 0.5,
                        0.5, 30, 0.8, 1
                ));

        waitForStart();

        forward.initialize();
        forward.schedule();

        while (opModeIsActive() && !isStopRequested()) {
            // control loop
            robot.setTargetSlidePos(RobotConfig.SlideHeight.LOW);
            robot.stackArm(RobotConfig.StackArm.IN);

        }

    }
}
