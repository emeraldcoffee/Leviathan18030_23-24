package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.testTeleOp;

@TeleOp
public class driveTeleOp extends OpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    Servo outtakeDrop;
    ElapsedTime dropTimer = new ElapsedTime();
    int liftPosition;

    @Override
    public void init() {
        frontLeft = hardwareMap.dcMotor.get("FL");
        //frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.dcMotor.get("BL");
        //backRight = hardwareMap.get(DcMotor.class, "BR");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        frontLeft.setPower(-gamepad1.left_stick_y);
        backLeft.setPower(-gamepad1.left_stick_y);

    }
}
