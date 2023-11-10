package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.Camera3BoxDetection;
import org.firstinspires.ftc.teamcode.pipelines.ColorMask;

@Autonomous
public class testingAuto extends LinearOpMode {
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive dt = new SampleMecanumDrive(hardwareMap);
        Camera3BoxDetection camBoxPipeline = new Camera3BoxDetection();
        ColorMask colorMaskPipeline = new ColorMask();
        HwMap hwMap = new HwMap();

        hwMap.init(hardwareMap);

        hwMap.transferMotor.setPower(-0.5);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            hwMap.transferMotor.setPower(-0.5);
        }
    }
}
