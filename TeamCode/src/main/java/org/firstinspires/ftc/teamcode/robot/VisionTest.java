package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.camera3BoxDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class VisionTest {
    OpenCvCamera webcam;
    camera3BoxDetection cam3BoxDetect;
    HardwareMap hw;

    hardwareMap hwMap = new hardwareMap();
    /*hwMap.init(hwMap);
    cam3BoxDetect = new camera3BoxDetection();

    hwMap.frontCamera.setPipeline(cam3BoxDetect);
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    hwMap.frontCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
    {
        public void onOpened()
        {
            hwMap.frontCamera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
        }

        @Override
        public void onError(int errorCode) {}
    });*/
}
