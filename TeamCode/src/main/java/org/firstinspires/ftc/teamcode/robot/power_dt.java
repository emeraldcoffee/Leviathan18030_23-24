package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Disabled
@TeleOp
public class power_dt extends LinearOpMode {
    public DcMotorEx leftFront;
    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            leftFront.setPower(1);
        }
        leftFront.setPower(0);


    }
}
