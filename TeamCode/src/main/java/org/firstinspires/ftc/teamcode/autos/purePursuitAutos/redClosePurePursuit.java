package org.firstinspires.ftc.teamcode.autos.purePursuitAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.AutoWayPoints;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.PassData;

@Autonomous
public class redClosePurePursuit extends LinearOpMode {
    enum PathControl {
        CAMERA,
        WAIT,
        INIT_PATH,
        CYCLE_STACK,
        CYCLE_BACKDROP
    }

    PathControl pathControl = PathControl.CAMERA;

    boolean init = true;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotConfig robot  = new RobotConfig(hardwareMap);
        robot.ResetSlides();

        Telemetry.Item detectedPos = telemetry.addData("Position", "No detection");
        Telemetry.Item IMU = telemetry.addData("Current IMU", robot.getCurrentIMU().toString());
        Telemetry.Item Park = telemetry.addData("Park Position", PassData.roadrunnerParkPosition.toString());

//        while (opModeInInit() && init) {
//            if (gamepad1.back) {
//                init = false;
//            }
//
//            telemetry.update();
//        }

        Path firstCycle = new Path(
                new StartWaypoint(52, -41.5),
                new GeneralWaypoint(PassData.crossBackdropSide.red, 10, .5, Math.toRadians(1)),
                new GeneralWaypoint(PassData.crossStackSide.red, 10, .5, Math.toRadians(1)),
                new EndWaypoint(PassData.stackPosition.red, DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, 10, .5, Math.toRadians(1))
        );


        Path placeCycle = new Path(
                new StartWaypoint(PassData.stackPosition.red),
                new GeneralWaypoint(PassData.crossStackSide.red, 10, .5, Math.toRadians(1)),
                new GeneralWaypoint(PassData.crossBackdropSide.red, 10, .5, Math.toRadians(1)),
                new EndWaypoint(PassData.crossBackdropSide.red, DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, 10, .5, Math.toRadians(1))
        );

        Path returnCycle = new Path(
                new StartWaypoint(52, -41.5),
                new GeneralWaypoint(PassData.crossBackdropSide.red, 10, .5, Math.toRadians(1)),
                new GeneralWaypoint(PassData.crossStackSide.red, 10, .5, Math.toRadians(1)),
                new EndWaypoint(PassData.stackPosition.red, DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, 10, .5, Math.toRadians(1))
        );

        Path park = new Path(
                new StartWaypoint(PassData.dropPosition.red),
                new GeneralWaypoint(AutoWayPoints.ParkPosition.BACK.red, 10, .5, Math.toRadians(1)),
                new EndWaypoint(PassData.parkPosition.red, DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, 10, .5, Math.toRadians(1))
        );



        waitForStart();
        robot.update();
        robot.setPoseEstimate(new Pose2d(52, -41.5, 0));

        robot.followPurePursuitPath(firstCycle);
        while (!robot.isPurePursuitPathFinished()) {
            robot.update();
        }
//        robot.isPurePursuitPathFinished();
//        while (!robot.isPurePursuitPathFinished()) {
//            robot.update();
//        }
//        robot.followPurePursuitPath(returnCycle);
//        while (!robot.isPurePursuitPathFinished()) {
//            robot.update();
//        }
//        robot.followPurePursuitPath(park);
//        while (!robot.isPurePursuitPathFinished()) {
//            robot.update();
//        }


//        while (!isStopRequested()) {
//        }

    }

}
