package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.ColorMask;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class BlueFrontStart extends LinearOpMode {

    String pos = "";

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive dt = new SampleMecanumDrive(hardwareMap);
        HwMap hwMap = new HwMap();
        ColorMask colorMaskPipeline = new ColorMask();
        ElapsedTime timer = new ElapsedTime();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        hwMap.webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"));
        hwMap.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {

            public void onOpened()
            {
                colorMaskPipeline.setAlliance("Blue");
                hwMap.webcam.setPipeline(colorMaskPipeline);
                pos = colorMaskPipeline.getPos();

                hwMap.webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error: ", errorCode);
            }
        });

        waitForStart();

        Pose2d bFStartPose = new Pose2d(-35, 63, Math.toRadians(270));
        Pose2d bBStartPose = new Pose2d(12, 63, Math.toRadians(270));
        Pose2d rFStartPose = new Pose2d(-35, -63, Math.toRadians(90));
        Pose2d rBStartPose = new Pose2d(12, -63, Math.toRadians(90));


        // in front of trusses on blue

        dt.setPoseEstimate(bFStartPose);
        TrajectorySequence blueFront = dt.trajectorySequenceBuilder(bFStartPose)
                .forward(50)
                // uses Vision to detect where the team prop is
                .addDisplacementMarker(() -> {
                    timer.reset();
                    if (pos.equals("left")) {

                        // for a certain amount of time or for motor encoders, strafe to the left and then drop, then return.
                        /*dt.trajectoryBuilder(new Pose2d())
                                .strafeLeft(30)
                                .build();

                        RobotMethods.outtakePlace(hwMap);
                        dt.trajectoryBuilder(new Pose2d())
                                .strafeRight(30)
                                .build();
                        */
                    }
                    else if (pos.equals("right")) {
                        // do the same thing for right
                        /*dt.trajectoryBuilder(new Pose2d())
                                .strafeRight(30)
                                .build();

                        RobotMethods.outtakePlace(hwMap);
                        dt.trajectoryBuilder(new Pose2d())
                                .strafeLeft(30)
                                .build();

                         */
                    }
                    else {
                        // do the same thing for center but just move forward, drop, then return
                        /*
                        dt.trajectoryBuilder(new Pose2d())
                                .forward(40)
                                .build();
                        RobotMethods.outtakePlace(hwMap);
                        dt.trajectoryBuilder(new Pose2d())
                                .back(40)
                                .build();

                         */
                    }
                })
                // places down pixel where team prop is
                .addDisplacementMarker(() -> {
                    // turn depending on where the team prop is
                    //RobotMethods.outtakePlace(hwMap);
                })
                .turn(Math.toRadians(90))
                //.splineTo(new Vector2d(-12, 35), Math.toRadians(0))
                .forward(90)
                // places down pixel on backdrop
                .addDisplacementMarker(() -> {
                    //RobotMethods.slideExtend(hwMap, 50);
                })
                // cycling
                /*.waitSeconds(1)
                .back(10)
                .turn(Math.toRadians(180))
                .forward(90)
                // picks up pixels
                .addDisplacementMarker(() -> {

                })
                .waitSeconds(1)
                .back(10)
                .turn(Math.toRadians(180))
                .forward(90)
                // place down pixel on backdrop
                .addDisplacementMarker(() -> {
                    //RobotMethods.slideExtend(hwMap, 50);
                })*/
                .build();

        waitForStart();
    
        if (!isStopRequested() && isStarted()) {
           dt.followTrajectorySequence(blueFront);
        }
    }

    public void leftDrop() {

    }
}
