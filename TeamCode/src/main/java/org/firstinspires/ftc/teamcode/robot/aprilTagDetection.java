package org.firstinspires.ftc.teamcode.robot;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
public class aprilTagDetection extends OpMode {
    //Creates aprilTagProcessor of type aprilTagProcessor
    //this processor is provided by the FTC sdk
    private AprilTagProcessor aprilTagProcessor;
    //VisionPortal is provided by the FTC SDK and is an easy way to handle camera feed
    private VisionPortal visionPortal;

    @Override
    public void init() {
        //Set to correct webcam, the second part
        WebcamName frontCamera = hardwareMap.get(WebcamName.class, "Webcam 1");
        //Creates the AprilTagProcessor with configured settings
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        //Creates visionPortal with configured setting, passes the webcam and the aprilTag Processor
        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .setCamera(frontCamera)
                //sets camera resolution to 640 by 480 so that we can use a default calibration
                .setCameraResolution(new Size(640,480))
                .build();
    }

    @Override
    //loops when you press initialize but before you press start
    /*will probably have to change this and put code that makes the robot
    move to the aprilTags and then detect them*/
    public void init_loop() {
        //Gets a list of aprilTags detected by the aprilTagProcessor
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        //Not type string because the is a lot of appending
        StringBuilder idsFound = new StringBuilder();
        //for-each loop that loops through the array of the detections
        for (AprilTagDetection detection : currentDetections)
        {
            idsFound.append(detection.id);
            idsFound.append(' ');
        }
        telemetry.addData("April Tags", idsFound);
    }

    @Override
    public void start() {
        visionPortal.stopStreaming();
    }

    @Override
    public void loop() {
    }
}
