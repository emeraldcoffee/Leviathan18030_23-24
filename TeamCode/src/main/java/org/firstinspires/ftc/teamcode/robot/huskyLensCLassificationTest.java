package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Sensor: HuskyLens", group = "Sensor")
public class huskyLensCLassificationTest extends LinearOpMode {

    HuskyLens huskyLens = hardwareMap.get(HuskyLens.class, "rightHuskyLens");

    @Override
    public void runOpMode() {
        waitForStart();

        while (opModeIsActive()) {

            huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_CLASSIFICATION);

            HuskyLens.Block[] blocks = huskyLens.blocks();

            telemetry.addData("Block count", blocks.length);

            for (int i = 0; i < blocks.length; i++) {
                if (blocks[i].id == 1) {

                }
                telemetry.addData("Block", blocks[i].toString());
            }

        }
    }
}
