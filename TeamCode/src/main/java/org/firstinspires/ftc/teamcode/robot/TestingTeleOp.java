package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class TestingTeleOp extends OpMode {

    SampleMecanumDrive dt;
    HwMap hwMap;

    @Override
    public void init() {
        dt = new SampleMecanumDrive(hardwareMap);
        hwMap = new HwMap();
        hwMap.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.right_trigger > 0.0) {
            //RobotMethods.slideExtend(hwMap, 5.0);
        }
    }
}
