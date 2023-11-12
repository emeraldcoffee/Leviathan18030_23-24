package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous()
public class StrafeRight extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        HwMap robot = new HwMap();
        robot.init(hardwareMap);
        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);

        robot.transferMotor.setPower(-.5);
        waitForStart();

        RobotMethods.setMecanumDrive(0, -1,0, 1, driveTrain);
        sleep(3000);

        RobotMethods.setMecanumDrive(0, 0,0, 1, driveTrain);
        robot.transferMotor.setPower(0);



    }


}
