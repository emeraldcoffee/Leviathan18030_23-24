package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.Camera3BoxDetection;
import org.firstinspires.ftc.teamcode.pipelines.ColorMask;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous
public class BlueFrontStart extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive dt = new SampleMecanumDrive(hardwareMap);
        Camera3BoxDetection camBoxPipeline = new Camera3BoxDetection();
        ColorMask colorMaskPipeline = new ColorMask();
        HwMap hwMap = new HwMap();

        hwMap.init(hardwareMap);

        hwMap.webcam.setPipeline(colorMaskPipeline);
        hwMap.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            public void onOpened()
            {
                hwMap.webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error: ", errorCode);
            }
        });

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

        }

        /*Pose2d bFStartPose = new Pose2d(-35, 63, Math.toRadians(270));
        Pose2d bBStartPose = new Pose2d(12, 63, Math.toRadians(270));
        Pose2d rFStartPose = new Pose2d(-35, -63, Math.toRadians(90));
        Pose2d rBStartPose = new Pose2d(12, -63, Math.toRadians(90));


        // in front of trusses on blue

        dt.setPoseEstimate(bFStartPose);
        TrajectorySequence blueFront = dt.trajectorySequenceBuilder(bFStartPose)
                .forward(19)
                // uses Vision to detect where the team prop is
                .addDisplacementMarker(() -> {

                })
                // places down pixel where team prop is
                .addDisplacementMarker(() -> {
                    // turn depending on where the team prop is
                    RobotMethods.outtakePlace(hwMap);
                })
                .splineTo(new Vector2d(-12, 35), Math.toRadians(0))
                .forward(60)
                // places down pixel on backdrop
                .addDisplacementMarker(() -> {
                    //RobotMethods.slideExtend(hwMap, 50);
                })
                // cycling
                .waitSeconds(1)
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
                })
                .build();

        waitForStart();

        if (!isStopRequested()) {
            dt.followTrajectorySequence(blueFront);
        }*/
    }
}