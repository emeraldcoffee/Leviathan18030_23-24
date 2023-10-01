package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class testDriveTrain {

    DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
    DcMotor leftRear = hardwareMap.get(DcMotor.class, "leftRear");
    DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
    DcMotor rightRear = hardwareMap.get(DcMotor.class, "rightRear");

    DcMotor lift = hardwareMap.get(DcMotor.class, "lift");
    Servo hatch = hardwareMap.get(Servo.class, "hatch");

    double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
    double x = gamepad1.left_stick_x;
    double rx = gamepad1.right_stick_x;

    public void setLeftFront(DcMotor lf) {
        lf.setPower(y + x + rx);
    }
    public void setLeftRear(DcMotor lr) {
        lr.setPower(y + x + rx);
    }
    public void setRightFront(DcMotor rf) {
        rf.setPower(y + x + rx);
    }
    public void setRightRear(DcMotor rr) {
        rr.setPower(y + x + rx);
    }
    public void setLift(DcMotor up) {
        up.setPower(gamepad1.right_trigger);
    }
}
