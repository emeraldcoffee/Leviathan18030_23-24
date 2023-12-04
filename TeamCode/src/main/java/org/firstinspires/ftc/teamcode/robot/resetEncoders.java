package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.util.Encoder;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous
public class resetEncoders extends LinearOpMode {
    public static DcMotorEx leftEncoder, rightEncoder, frontEncoder, liftEncoder;

    @Override
    public void runOpMode() {
        leftEncoder = hardwareMap.get(DcMotorEx.class, "slideMotor");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "rightFront");
        frontEncoder = hardwareMap.get(DcMotorEx.class, "rightRear");
        liftEncoder = hardwareMap.get(DcMotorEx.class, "climbMotor");

        waitForStart();
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
}


