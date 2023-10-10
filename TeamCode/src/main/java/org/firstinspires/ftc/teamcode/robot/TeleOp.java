package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class TeleOp extends OpMode {

    @Override
    public void init() {
        HardwareMap hwMap = new HardwareMap();
        SampleMecanumDrive dt = new SampleMecanumDrive(hwMap);


    }

    @Override
    public void loop() {

    }
}
