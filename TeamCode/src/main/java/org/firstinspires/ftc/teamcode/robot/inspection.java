package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp
public class inspection extends LinearOpMode {
    public DcMotorEx intakeMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            intakeMotor.setPower(.7);
        }
        intakeMotor.setPower(0);


    }
}
