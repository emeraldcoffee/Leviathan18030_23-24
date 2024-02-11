package org.firstinspires.ftc.teamcode.robot;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp
public class velocityTest extends LinearOpMode {
    Encoder left, right, center;


    @SuppressLint("DefaultLocale")
    public void runOpMode() {

        left = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
        right = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));
        center = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear"));

        Telemetry.Item endoderPositions = telemetry.addData("Encoder Positions", "");
        Telemetry.Item endoderVelocities = telemetry.addData("Encoder Velocities", "");


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            endoderPositions.setValue(String.format("Left: %,6d Right: %,6d Center: %,6d", left.getCurrentPosition(), right.getCurrentPosition(), center.getCurrentPosition()));
            endoderVelocities.setValue(String.format("Left: %,5.2f Right: %,5.2f Center: %,5.2f", left.getCorrectedVelocity(), right.getCorrectedVelocity(), center.getCorrectedVelocity()));

            telemetry.update();
        }

    }
}
