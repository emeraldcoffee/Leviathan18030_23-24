package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.Camera3BoxDetection;
import org.firstinspires.ftc.teamcode.pipelines.ColorMask;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp (name = "CameraTest", group = "testing")
public class Vision extends LinearOpMode {
    String pos = "";
    @Override
    public void runOpMode() throws InterruptedException {

        //SampleMecanumDrive dt = new SampleMecanumDrive(hardwareMap);
        ColorMask colorMaskPipeline = new ColorMask();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"));
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {

            public void onOpened()
            {
                colorMaskPipeline.setAlliance("Red");
                webcam.setPipeline(colorMaskPipeline);
                pos = colorMaskPipeline.getPos();
                System.out.println("Position: " + pos);
                telemetry.addData("Position: ", pos);

                webcam.startStreaming(640,480, OpenCvCameraRotation.UPSIDE_DOWN);
            }



            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error: ", errorCode);
            }
        });

        waitForStart();

        while (isStarted() && !isStopRequested()) {
            telemetry.addData("Position: ", pos);
        }
    }
}
