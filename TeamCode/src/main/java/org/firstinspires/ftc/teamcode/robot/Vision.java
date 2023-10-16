package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.TestPipeline;
import org.firstinspires.ftc.teamcode.pipelines.camera3BoxDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp (name = "CameraTest", group = "testing")
public class Vision extends LinearOpMode {
    OpenCvCamera webcam;
    camera3BoxDetection Camera3BoxDetection;
    HardwareMap hw;

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareMap robot = new hardwareMap();
        robot.init(hardwareMap);
        Camera3BoxDetection = new camera3BoxDetection();

        robot.frontCamera.setPipeline(Camera3BoxDetection);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

//        WebcamName webcamName = hardwareMap.get(WebcamName.class, "camera");
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        robot.frontCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                robot.frontCamera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

        }
    }
}
