package org.firstinspires.ftc.teamcode.robot;

import com.google.blocks.ftcrobotcontroller.runtime.Block;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorHuskyLens;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "huskylens")
public class huskyLensTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);
        HwMap robot = new HwMap();
        robot.init(hardwareMap);

        robot.huskylens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_CLASSIFICATION);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            for (HuskyLens.Block b : robot.huskylens.blocks()) {
                int objID = b.id;
                // save the current recognition's Color ID
                telemetry.addData("This object ID", objID);
                if (objID == 2) {
                    telemetry.addData("Object 2 detected!", "");
                }
                else if (objID == 3) {
                    telemetry.addData("Object 3 detected!", "");
                    }
                else {
                    telemetry.addData("nothing detected!", "");
                telemetry.update();
                }

            }
            telemetry.update();




        }
    }

}
