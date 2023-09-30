package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pipelines.testPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp (name = "CameraTest", group = "testing")
public class Vision extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        OpenCvCamera webcam;
        HardwareMap hw;

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam.setPipeline(new testPipeline());

        waitForStart();

        while (opModeIsActive()) {
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                public void onOpened() {
                    webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
                    // Usually this is where you'll want to start streaming from the camera (see section 4)
                }

                public void onError(int errorCode) {
                    telemetry.addLine("Something's wrong!");
                }
            });
        }
    }
}
