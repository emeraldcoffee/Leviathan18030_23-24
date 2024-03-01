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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autos.redFar2plus1;
import org.firstinspires.ftc.teamcode.drive.AutoWayPoints;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.RobotConfig;
import org.firstinspires.ftc.teamcode.pipelines.ColorMask;
import org.firstinspires.ftc.teamcode.robot.PassData;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class redClosePurePursuit extends LinearOpMode {
    //How long it take the robot to complete 1 cycle
    double cycleTime = 10;

    //How long it takes the robot to park
    double parkTime = 1;
    String pos = "";

    enum PathControl {
        CAMERA,
        WAIT,
        INIT_PATH,
        CYCLE_STACK,
        CYCLE_BACKDROP,
        FINISHED
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

        ElapsedTime runTimer = new ElapsedTime();


        ColorMask pipeline = new ColorMask();

        robot.webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"));
        robot.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                pipeline.setAlliance("Red");
                robot.webcam.setPipeline(pipeline);

                robot.webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error: ", errorCode);
            }
        });

//        while (opModeInInit() && init) {
//            if (gamepad1.dpad_left) {
//                PassData.stackPosition = AutoWayPoints.StackPosition.WALL;
//                PassData.dropPosition = AutoWayPoints.DropPosition.WALL;
//                PassData.crossStackSide = AutoWayPoints.CrossStackSide.WALL;
//                PassData.crossBackdropSide = AutoWayPoints.CrossBackdropSide.WALL;
//                PassData.parkPosition = AutoWayPoints.ParkPosition.CENTER;
//            } else if ()
//
//            if (gamepad1.back) {
//                init = false;
//            }
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
        runTimer.reset();

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

        if (true) return;

        while (!isStopRequested()) {
            switch (pathControl) {
                case CAMERA:
                    //Saves camera reading
                    if (runTimer.seconds()>.5) {
                        pos = pipeline.getPos();
                        pathControl = PathControl.WAIT;
                    }
                    break;
                case WAIT:
                    //Use to make auto wait for alliance partner
                    if (runTimer.seconds()>.5) {
                        switch (pos) {
                            case "left":
//                                robot.followTrajectorySequenceAsync(left);
                                detectedPos.setValue("Left");
                                break;
                            case "center":
//                                robot.followTrajectorySequenceAsync(center);
                                detectedPos.setValue("Center");
                                break;
                            case "right":
//                                robot.followTrajectorySequenceAsync(right);
                                detectedPos.setValue("Right");
                                break;
                            default:
//                                robot.followTrajectorySequenceAsync(center);
                                detectedPos.setValue("Default center (No detection)");
                                break;
                        }
                        pathControl = PathControl.INIT_PATH;
                    }
                    break;
                case INIT_PATH:
                    if (!robot.isBusy()) {
                        returnCycle.set(0, new StartWaypoint(robot.getPoseEstimate().getX(), robot.getPoseEstimate().getY()));
                        robot.followPurePursuitPath(returnCycle);
                        pathControl = PathControl.CYCLE_BACKDROP;
                    }
                    break;
                case CYCLE_BACKDROP:
                    if (robot.isPurePursuitPathFinished()) {
                        robot.followPurePursuitPath(placeCycle);
                        pathControl = PathControl.CYCLE_STACK;
                    }
                    break;
                case CYCLE_STACK:
                    if (robot.isPurePursuitPathFinished()) {
                        if (runTimer.seconds() < 30 - cycleTime){
                            robot.followPurePursuitPath(returnCycle);
                            pathControl = PathControl.CYCLE_BACKDROP;
                        } else if (runTimer.seconds() < 30 - parkTime) {
                            robot.followPurePursuitPath(park);
                            pathControl = PathControl.FINISHED;
                        } else {
                            pathControl = PathControl.FINISHED;
                        }
                    }
                        break;
            }

            robot.update();
            telemetry.update();
        }

    }

}
