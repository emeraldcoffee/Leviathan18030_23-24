package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.RobotConstants.slidePIDVals;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class SimpleAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        HwMap robot = new HwMap();
        robot.init(hardwareMap);
        SampleMecanumDrive dt = new SampleMecanumDrive(hardwareMap);

        int targetPos = RobotConstants.slideBottom;
        double slideI = 0;

        robot.transferMotor.setPower(-.2);
        targetPos = RobotConstants.slideGround;
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            RobotMethods.setMecanumDrive(1, 0, 0, dt);
            sleep(300);
            targetPos = RobotConstants.slideLow;
            robot.dropServo.setPosition(RobotConstants.dropOpen);
            sleep((long) RobotConstants.dropTime * 1000);
            robot.dropServo.setPosition(RobotConstants.dropClosed);
            targetPos = RobotConstants.slideGround;
            sleep(300);
            RobotMethods.setMecanumDrive(0, -1, 0, dt);
            sleep(300);
            RobotMethods.setMecanumDrive(0, 0, 0.5, dt);
            targetPos = RobotConstants.slideLow;
            robot.dropServo.setPosition(RobotConstants.dropOpen);
            sleep((long) RobotConstants.dropTime * 1000);
            robot.dropServo.setPosition(RobotConstants.dropClosed);


            double slideVelo = robot.liftEncoder.getCorrectedVelocity();
            int slideCurPos = robot.liftEncoder.getCurrentPosition();

            double distRemain = targetPos - slideCurPos;

            slideI += distRemain * slidePIDVals.i;

            double slidePower = (distRemain * slidePIDVals.p) + slideI + (slideVelo * slidePIDVals.d);

            robot.liftMotor.setPower(slidePower);

        }
    }
}
