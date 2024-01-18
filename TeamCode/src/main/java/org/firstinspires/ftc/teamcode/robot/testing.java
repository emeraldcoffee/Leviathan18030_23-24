package org.firstinspires.ftc.teamcode.robot;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class testing extends LinearOpMode {

    public WebcamName frontCamera;

    public void runOpMode() {
        //Init code
//        HwMap robot = new HwMap();
//        robot.init(hardwareMap);
//        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);

        frontCamera = hardwareMap.get(WebcamName.class, "camera");


        AprilTagProcessor frontAprilTagProcessor;
        VisionPortal frontVisionPortal;
        frontAprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();


        //Creates visionPortal with configured setting, passes the webcam and the aprilTag Processor
        frontVisionPortal = new VisionPortal.Builder()
                .addProcessor(frontAprilTagProcessor)
                .setCamera(frontCamera)
                //sets camera resolution to 640 by 480 so that we can use a default calibration
                .setCameraResolution(new Size(640,480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
        AprilTagDetection frontCamAprilTags;

        ElapsedTime runTime = new ElapsedTime();
        Telemetry.Item aprilTagPosEstimate = telemetry.addData("Apriltag Estimate", "");
//        Telemetry.Item averageBuildTime = telemetry.addData("Average Build Time", "Running");

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            if (frontAprilTagProcessor.getDetections().size() > 0) {
                //Gets all the april tag data for the 1st detection
                frontCamAprilTags = frontAprilTagProcessor.getDetections().get(0);
                aprilTagPosEstimate.setValue(RobotMethods.updateRobotPosAprilTag(frontCamAprilTags));
//                tagDetection = true;
            } else {
                aprilTagPosEstimate.setValue("No tags detected");
//                tagDetection = false;
            }
            telemetry.update();
        }

//        runTime.reset();
//        for (int i = 0; i <1000; i++) {
//            Trajectory testLine = driveTrain.trajectoryBuilder(new Pose2d(4, 4, Math.toRadians(30)))
//                    .lineToSplineHeading(new Pose2d(53, 37.5, Math.toRadians(0)))
//                    .build();
//        }
//        averageBuildTime.setValue(runTime.milliseconds()/1000);



    }


}
