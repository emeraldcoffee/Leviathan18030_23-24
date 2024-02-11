package org.firstinspires.ftc.teamcode.robot;

import static java.lang.Math.abs;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.RobotConfig;

import java.util.Timer;

public class newTele extends LinearOpMode {

    enum DriveStates {
        CorrectiveTurning,
        FieldCentric,
        BasicTurning
    }
    testTele.DriveStates driveStates = testTele.DriveStates.CorrectiveTurning;

    double lastTurn = 0;

    enum Climb {
        HOLD,
        RELEASE,
        WAIT,
        STOPPED,
    }
    Climb climb = Climb.HOLD;

    enum Drop {
        OPEN,
        DOUBLE_OPEN,
        CLOSED,
        RESET
    }
    Drop drop = Drop.CLOSED;

    enum SpikeMark {
        GUIDE,
        OUT,
        HOLD,
        IN
    }
    SpikeMark spikemark = SpikeMark.GUIDE;

    enum Spin {
        STOPPED,
        SPIN_IN,
        SPIN_OUT
    }
    Spin transfer = Spin.STOPPED;
    Spin intake = Spin.STOPPED;

    double targetClimbPos = 0;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(false);
        Telemetry.Item status = telemetry.addData("Status", "Initializing");
        telemetry.update();

        RobotConfig robot = new RobotConfig(hardwareMap);

        Telemetry.Item robotPose = telemetry.addData("Robot pose", RobotMethods.updateRobotPosition(robot.getPoseEstimate()));
        Telemetry.Item loopTime = telemetry.addData("Loop time", "");


        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime climbTimer = new ElapsedTime();
        ElapsedTime dropTimer = new ElapsedTime();
        ElapsedTime slideTimer = new ElapsedTime();

        //Creating gamepads
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad prevGamepad1 = new Gamepad();
        Gamepad prevGamepad2 = new Gamepad();

        status.setValue("Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        status.setValue("Running");

        while (opModeIsActive() && !isStopRequested()) {
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            double finalSpeed = RobotConstants.speedMultiplier * (1 + (currentGamepad1.right_trigger - (currentGamepad1.left_trigger)*.4) / 1.2);

            switch (driveStates) {
                case BasicTurning:
                    robot.setMecanumDrive(-currentGamepad1.left_stick_y * RobotConstants.driveSpeed * finalSpeed,
                            -currentGamepad1.left_stick_x * RobotConstants.strafeSpeed * finalSpeed
                            , -currentGamepad1.right_stick_x * RobotConstants.turnSpeed);
                    break;
            }

            //Climb code
            switch (climb) {
                case HOLD:
                    if (currentGamepad1.dpad_down && currentGamepad1.right_bumper) {
                        targetClimbPos = robot.climbMotor.getCurrentPosition() + 200;
                        robot.climbMotor.setTargetPosition((int) targetClimbPos);
                        climbTimer.reset();
                        climb = Climb.RELEASE;
                    }
                    break;
                case RELEASE:
                    if (climbTimer.milliseconds()>RobotConstants.climbReleaseDelay && !gamepad1.dpad_down) {
                        climb = Climb.WAIT;
                    }
                    break;
                case WAIT:
                    if (currentGamepad1.dpad_down) {
                        driveStates = testTele.DriveStates.BasicTurning;
                        climbTimer.reset();
                        climb = Climb.STOPPED;
                    }
                    break;
                case STOPPED:
                    if (currentGamepad1.dpad_down) {
                        targetClimbPos +=  2.5*climbTimer.milliseconds();
                        robot.climbMotor.setTargetPosition((int) targetClimbPos);
                    } else if (currentGamepad1.dpad_up) {
                        targetClimbPos -=  2.5*climbTimer.milliseconds();
                        robot.climbMotor.setTargetPosition((int) targetClimbPos);
                    }
                    climbTimer.reset();
                    break;
            }

            //Drone Launch
            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                robot.releaseDrone();
            }


            //Driver 2 code
            //Code for dropping pixels out of outtake
            switch (drop) {
                case CLOSED:
                    if (currentGamepad2.left_bumper) {
                        robot.dropper(RobotConfig.Dropper.OPEN);
                        dropTimer.reset();
                        drop = Drop.OPEN;
                    } else if (currentGamepad2.right_bumper) {
                        robot.dropper(RobotConfig.Dropper.PARTIAL);
                        dropTimer.reset();
                        drop = Drop.DOUBLE_OPEN;
                    }
                    break;
                case OPEN:
                    if (dropTimer.seconds() > RobotConstants.dropTime) {
                        robot.dropper(RobotConfig.Dropper.CLOSED);
                        dropTimer.reset();
                        drop = Drop.RESET;
                    }
                    break;
                case DOUBLE_OPEN:
                    if (dropTimer.seconds() > RobotConstants.doubleDropTime) {
                        robot.dropper(RobotConfig.Dropper.CLOSED);
                        dropTimer.reset();
                        drop = Drop.RESET;
                    }
                    break;
                case RESET:
                    if (dropTimer.seconds() > RobotConstants.resetTime) {
                        drop = Drop.CLOSED;
                    }
                    break;
                default:
                    drop = Drop.RESET;
            }

            //Slide code
            if (abs(currentGamepad2.left_stick_y)>.1) {
                robot.setTargetSlidePos(robot.getTargetSlidePos()-currentGamepad2.left_stick_y*slideTimer.milliseconds());
            } else if (currentGamepad2.a) {
                robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
            } else if (currentGamepad2.x) {
                robot.setTargetSlidePos(RobotConfig.SlideHeight.LOW);
            } else if (currentGamepad2.b) {
                robot.setTargetSlidePos(RobotConfig.SlideHeight.MEDIUM);
            } else if (currentGamepad2.y) {
                robot.setTargetSlidePos(RobotConfig.SlideHeight.HIGH);
            }

            slideTimer.reset();

            //Intake code
            switch (spikemark) {
                case GUIDE:
                    if (currentGamepad2.left_trigger > .1) {
                        robot.stackArm(RobotConfig.StackArm.OUT);
                        spikemark = SpikeMark.OUT;
                    }
                    break;
                case OUT:
                    if (currentGamepad2.left_trigger > .9) {
                        robot.stackHold(true);
                        spikemark = SpikeMark.HOLD;
                    } else if (currentGamepad2.left_trigger < .1) {
                        robot.stackArm(RobotConfig.StackArm.GUIDE);
                        spikemark = SpikeMark.GUIDE;
                    }
                    break;
                case HOLD:
                    if (currentGamepad2.right_trigger > .1) {
                        robot.stackArm(RobotConfig.StackArm.IN);
                        spikemark = SpikeMark.IN;
                    } else if (currentGamepad2.left_trigger < .9) {
                        robot.stackHold(false);
                        spikemark = SpikeMark.OUT;
                    }
                    break;
                case IN:
                    if (currentGamepad2.right_trigger < .1) {
                        robot.stackArm(RobotConfig.StackArm.OUT);
                        spikemark = SpikeMark.HOLD;
                    }
                    break;
            }

            switch (intake) {
                case STOPPED:
                    if (currentGamepad2.dpad_down || currentGamepad2.left_trigger >.1) {
                        robot.intakeMotor.setPower(RobotConstants.intakeSpeed);
                        intake = Spin.SPIN_IN;
                    } else if (currentGamepad2.dpad_up) {
                        robot.intakeMotor.setPower(-RobotConstants.intakeSpeed);
                        intake = Spin.SPIN_OUT;
                    }
                    break;
                case SPIN_IN:
                    if (!currentGamepad2.dpad_down || currentGamepad2.left_trigger <.1) {
                        robot.intakeMotor.setPower(0);
                        intake = Spin.STOPPED;
                    }
                    break;
                case SPIN_OUT:
                    if (!currentGamepad2.dpad_up) {
                        robot.intakeMotor.setPower(0);
                        intake = Spin.STOPPED;
                    }
                    break;
            }

            switch (transfer) {
                case STOPPED:
                    if (currentGamepad2.dpad_right || currentGamepad2.left_trigger >.1) {
                        robot.transferMotor.setPower(RobotConstants.transferSpeed);
                        transfer = Spin.SPIN_IN;
                    } else if (currentGamepad2.dpad_left) {
                        robot.transferMotor.setPower(-RobotConstants.transferSpeed);
                        transfer = Spin.SPIN_OUT;
                    }
                    break;
                case SPIN_IN:
                    if (!currentGamepad2.dpad_right || currentGamepad2.left_trigger <.1) {
                        robot.transferMotor.setPower(0);
                        transfer = Spin.STOPPED;
                    }
                    break;
                case SPIN_OUT:
                    if (!currentGamepad2.dpad_left) {
                        robot.transferMotor.setPower(0);
                        transfer = Spin.STOPPED;
                    }
                    break;
            }

            robot.update();

            prevGamepad1.copy(currentGamepad1);
            prevGamepad2.copy(currentGamepad2);

            loopTime.setValue(String.format("%,3.2f ms", loopTimer.milliseconds()));
            loopTimer.reset();
            telemetry.update();
        }


    }

}
