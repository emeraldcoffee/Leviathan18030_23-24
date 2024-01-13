package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class inspection extends LinearOpMode {
    public Servo leftLiftServo, rightLiftServo;
    @Override
    public void runOpMode() throws InterruptedException {
        leftLiftServo = hardwareMap.servo.get("leftLiftServo");// control hub 1
        rightLiftServo = hardwareMap.servo.get("rightLiftServo");
        waitForStart();
        leftLiftServo.setPosition(RobotConstants.stackMax + RobotConstants.stackLeftOffset);
        rightLiftServo.setPosition(RobotConstants.stackMax);
        while (opModeIsActive() && !isStopRequested()) {
            sleep(5);
        }


    }
}
