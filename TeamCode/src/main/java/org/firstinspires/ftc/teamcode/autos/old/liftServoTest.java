package org.firstinspires.ftc.teamcode.autos.old;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.HwMap;

@TeleOp
public class liftServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        HwMap robot = new HwMap();
        robot.init(hardwareMap);
        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);
        telemetry.addData("Right Servo Pos", robot.rightLiftServo.getPosition());
        telemetry.addData("Left Servo Pos", robot.leftLiftServo.getPosition());


        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.update();
        }
    }
}
