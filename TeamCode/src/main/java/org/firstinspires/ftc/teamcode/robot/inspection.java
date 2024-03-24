package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class inspection extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        HwMap robot = new HwMap();
        robot.init(hardwareMap);

        robot.spikeMarkHoldServo.setPosition(RobotConstants.holdServoUp);

        waitForStart();

        robot.spikeMarkHoldServo.setPosition(RobotConstants.holdServoUp);
        robot.transferMotor.setPower(-1);

        while (opModeIsActive() && !isStopRequested()) {
            sleep(5);
        }


    }
}
