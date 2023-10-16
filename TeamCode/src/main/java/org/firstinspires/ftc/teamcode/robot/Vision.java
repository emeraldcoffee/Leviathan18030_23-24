package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pipelines.TestPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp (name = "CameraTest", group = "testing")
public class Vision extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        OpenCvCamera webcam;
        HardwareMap hw;

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);



        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                public void onOpened() {
                    webcam.setPipeline(new TestPipeline());
                    webcam.startStreaming(180, 360, OpenCvCameraRotation.UPRIGHT);
                    FtcDashboard.getInstance().startCameraStream(webcam, 5);
                    // Usually this is where you'll want to start streaming from the camera (see section 4)
                }

                public void onError(int errorCode) {
                    telemetry.addLine("Something's wrong! " + errorCode);
                }
            });
        }
    }
}
