package org.firstinspires.ftc.teamcode.robot;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp()
public class judgeTele extends LinearOpMode {

    enum Climb {
        HOLD,
        RELEASE,
        WAIT,
        STOPPED,
    }

    Climb climb = Climb.HOLD;

    enum Spin {
        STOPPED,
        SPIN_IN,
        SPIN_OUT
    }

    Spin transfer = Spin.STOPPED;
    Spin intake = Spin.STOPPED;

    enum Slide {
        BOTTOM,
        LOW,
        MIDDLE,
        TOP
    }

    enum SpikeMark {
        GUIDE,
        OUT,
        HOLD,
        IN
    }
    SpikeMark spikemark = SpikeMark.GUIDE;

    int targetPos = RobotConstants.slideBottom;

    double slideI = 0.0;

    int targetClimbPos;

    @Override
    public void runOpMode() throws InterruptedException {

        HwMap robot = new HwMap();
        robot.init(hardwareMap);
        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);

        ElapsedTime dropTimer = new ElapsedTime();

        ElapsedTime climbTimer = new ElapsedTime();

        ElapsedTime slideTimer = new ElapsedTime();

        ElapsedTime loopTimer = new ElapsedTime();

        robot.dropServo.setPosition(RobotConstants.dropClosed);

        robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide + RobotConstants.rightSpikeOffset);
        robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide);
        robot.spikeMarkHoldServo.setPosition(RobotConstants.holdServoUp);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            loopTimer.reset();

            //Code to change target position of slides
            if (abs(gamepad2.left_stick_y)>.1) {
                targetPos -= 8 * gamepad2.left_stick_y*slideTimer.milliseconds();
                targetPos = Range.clip(targetPos, RobotConstants.slideBottom, RobotConstants.slideTop);
            } else if (gamepad2.a) {
                targetPos = RobotConstants.slideBottom;
            } else if (gamepad2.x) {
                targetPos = RobotConstants.slideLow;
            } else if (gamepad2.b) {
                targetPos = RobotConstants.slideMiddle;
            } else if (gamepad2.y) {
                targetPos = RobotConstants.slideTop;
            }

            slideTimer.reset();
            double slideVelo = robot.liftEncoder.getCorrectedVelocity();
            int slideCurPos = robot.liftEncoder.getCurrentPosition();

            double distRemain = targetPos - slideCurPos;

            slideI += distRemain * RobotConstants.slidePIDVals.i;

            double slidePower = (distRemain * RobotConstants.slidePIDVals.p) + slideI + (slideVelo * RobotConstants.slidePIDVals.d);

            robot.slideMotor.setPower(slidePower);

            switch (climb) {
                case HOLD:
                    if (gamepad2.right_bumper) {
                        targetClimbPos = robot.climbMotor.getCurrentPosition() + 200;
                        robot.climbMotor.setTargetPosition(targetClimbPos);
                        climbTimer.reset();
                        climb = Climb.RELEASE;
                    }
                    break;
                case RELEASE:
                    if (climbTimer.milliseconds() > RobotConstants.climbReleaseDelay) {
//                        robot.climbMotor.setPower(0);
                        climb = Climb.WAIT;
                    }
                    break;
            }

            switch (intake) {
                case STOPPED:
                    if (gamepad2.dpad_down || gamepad2.left_trigger >.1) {
                        robot.intakeMotor.setPower(RobotConstants.intakeSpeed);
                        intake = Spin.SPIN_IN;
                    }
                    break;
                case SPIN_IN:
                    if (!gamepad2.dpad_down || gamepad2.left_trigger <.1) {
                        robot.intakeMotor.setPower(0);
                        intake = Spin.STOPPED;
                    }
                    break;
            }

            switch (transfer) {
                case STOPPED:
                    if (gamepad2.dpad_right) {
                        robot.transferMotor.setPower(RobotConstants.transferSpeed);
                        transfer = Spin.SPIN_IN;
                    }
                    break;
                case SPIN_IN:
                    if (!gamepad2.dpad_right) {
                        robot.transferMotor.setPower(0);
                        transfer = Spin.STOPPED;
                    }
                    break;
            }

            switch (spikemark) {
                case GUIDE:
                    if (gamepad2.left_trigger > .1) {
                        robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack);
                        robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack + RobotConstants.rightSpikeOffset);
                        spikemark = SpikeMark.OUT;
                    }
                    break;
                case OUT:
                    if (gamepad2.left_trigger > .9) {
                        robot.spikeMarkHoldServo.setPosition(RobotConstants.holdServoDown);
                        spikemark = SpikeMark.HOLD;
                    } else if (gamepad2.left_trigger < .1) {
                        robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide);
                        robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide + RobotConstants.rightSpikeOffset);
                        spikemark = SpikeMark.GUIDE;
                    }
                    break;
                case HOLD:
                    if (gamepad2.right_trigger > .1) {
                        robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                        robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn + RobotConstants.rightSpikeOffset );
                        spikemark = SpikeMark.IN;
                    } else if (gamepad2.left_trigger < .9) {
                        robot.spikeMarkHoldServo.setPosition(RobotConstants.holdServoUp);
                        spikemark = SpikeMark.OUT;
                    }
                    break;
                case IN:
                    if (gamepad2.right_trigger < .1) {
                        robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack);
                        robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkBack + RobotConstants.rightSpikeOffset);
                        spikemark = SpikeMark.HOLD;
                    }
                    break;
            }

            switch (transfer) {
                case STOPPED:
                    if (gamepad2.dpad_right || gamepad2.left_trigger >.1) {
                        robot.transferMotor.setPower(RobotConstants.transferSpeed);
                        transfer = Spin.SPIN_IN;
                    }
                    break;
                case SPIN_IN:
                    if (!gamepad2.dpad_right || gamepad2.left_trigger <.1) {
                        robot.transferMotor.setPower(0);
                        transfer = Spin.STOPPED;
                    }
                    break;
            }

            if (gamepad2.left_bumper) {
                robot.droneServo.setPosition(RobotConstants.droneRelease);
            }

            if (gamepad2.start) {
                robot.leftServo.setPosition(RobotConstants.leftOut);
            }
            if (gamepad2.back) {
                robot.leftServo.setPosition(RobotConstants.leftIn);
            }
        }
        robot.slideMotor.setPower(0.0);
    }
}
