package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class sample extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor motorTest;
    private DigitalChannel digitalTouch;
    private DistanceSensor sensorColorRange;
    private Servo servoTest;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(Gyroscope.class, "imu");
        motorTest = hardwareMap.get(DcMotor.class, "motorTest");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "motorTest");
        sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        servoTest = hardwareMap.get(Servo.class, "servoTest");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for game to start
        waitForStart();
        //run until end of match
        double tgtPower;
//        while (opModeIsActive()) {
//            if (digitalTouch.getState() == false) telemetry.addData("Button", "PRESSED");
//            else telemetry.addData("BUTTON", "NOT PRESSED");
//            telemetry.addData("Status", "Running");
//            telemetry.update();
//
//            tgtPower = -this.gamepad1.left_stick_y;
//            motorTest.setPower(tgtPower);
//            if(gamepad1.y) servoTest.setPosition(0);
//            else if (gamepad1.x ||gamepad1.b) servoTest.setPosition(0.5);
//            else if(gamepad1.a) servoTest.setPosition(1);
//            }
//            telemetry.addData("Servo Position", servoTest.getPosition());
//            telemetry.addData("Target Power", tgtPower);
//            telemetry.addData("Motor Power", motorTest.getPower());
//            telemetry.addData("Distance (cm)", sensorColorRange.getDistance(DistanceUnit.CM));
//            telemetry.addData("Status", "Running");
//            telemetry.update();
//
//        }
    }
}

