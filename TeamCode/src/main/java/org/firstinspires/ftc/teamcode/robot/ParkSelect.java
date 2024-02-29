package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class ParkSelect extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(false);
        Telemetry.Item parkPos = telemetry.addData("Park Position", PassData.roadrunnerParkPosition.toString());
        telemetry.update();


        waitForStart();
        while (!isStopRequested()) {
            if (gamepad1.x) {
                PassData.roadrunnerParkPosition = RobotConstants.ParkPosition.WALL;
            } else if (gamepad1.b) {
                PassData.roadrunnerParkPosition = RobotConstants.ParkPosition.CENTER;
            }

            parkPos.setValue(PassData.roadrunnerParkPosition.toString());
            telemetry.update();
        }
    }
}
