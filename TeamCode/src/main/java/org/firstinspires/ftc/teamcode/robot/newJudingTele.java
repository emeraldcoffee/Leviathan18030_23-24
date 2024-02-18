package org.firstinspires.ftc.teamcode.robot;

import static java.lang.Math.abs;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.RobotConfig;


@TeleOp
public class newJudingTele extends LinearOpMode {

    enum DriveStates {
        CorrectiveTurning,
        FieldCentric,
        BasicTurning
    }
    DriveStates driveStates = DriveStates.CorrectiveTurning;
    double targetHeading = 0;


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

        Telemetry.Item driveState = telemetry.addData("Drive State", "Corrective Turning");
        Telemetry.Item turnVals = telemetry.addData("Turn vals:", "");
        Telemetry.Item otherIMU = telemetry.addData("Other IMU vals", "");
        Telemetry.Item currentIMU = telemetry.addData("IMU", "");
        Telemetry.Item robotPose = telemetry.addData("Robot pose", RobotMethods.updateRobotPosition(robot.getPoseEstimate()));
        Telemetry.Item odomPositions = telemetry.addData("Encoder Positoions", "");
//        Telemetry.Item imuAngle = telemetry.addData("IMU rotation", "");
        Telemetry.Item loopTime = telemetry.addData("Loop time", "");


        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime driveTrainTimer = new ElapsedTime();
        ElapsedTime climbTimer = new ElapsedTime();
        ElapsedTime dropTimer = new ElapsedTime();
        ElapsedTime slideTimer = new ElapsedTime();

        //Creating gamepads
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad prevGamepad1 = new Gamepad();
        Gamepad prevGamepad2 = new Gamepad();

        robot.rightPixelServo.setPosition(RobotConstants.rightOut);

        status.setValue("Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        status.setValue("Running");

        loopTimer.reset();
        slideTimer.reset();

        while (!isStopRequested()) {
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

//            double finalSpeed = RobotConstants.speedMultiplier * (1 + (currentGamepad1.right_trigger - (currentGamepad1.left_trigger)*.4) / 1.2);
//
//            if (abs(currentGamepad1.right_stick_x)>.05) {
//                targetHeading = RobotMethods.fastInRangeRad( targetHeading-currentGamepad1.right_stick_x*Math.abs(currentGamepad1.right_stick_x) * RobotConstants.turnSpeed * driveTrainTimer.seconds()*5
//                );
//            }
//            driveTrainTimer.reset();
//            double vel = 0;
//            if (robot.getPoseVelocity() != null) {
//                vel = robot.getPoseVelocity().getHeading();
//            }
//
//            double headingComponent = Range.clip(RobotMethods.fastAngleDifferenceRad(targetHeading,robot.getPoseEstimate().getHeading())*1.1, -1, 1)
//                    + vel*.00015;
////            YawPitchRollAngles expansionRot = robot.expansionIMU.getRobotYawPitchRollAngles();
//            turnVals.setValue(String.format("%,3.2f Velocity %,3.2f", robot.expansionIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), robot.expansionIMU.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate));
////            Yaw: %,3.2f Pitch: %,3.2f Roll: %,3.2f Velocity %,3.2f
//            otherIMU.setValue(String.format("%,3.2f Velocity %,3.2f", robot.controlIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), robot.controlIMU.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate));
//
//            currentIMU.setValue(robot.getCurrentIMU().toString());//java.lang.Enum.name(robot.getCurrentIMU())
//
//            switch (driveStates) {
//                case BasicTurning:
//                    robot.setMecanumDrive(-currentGamepad1.left_stick_y * RobotConstants.driveSpeed * finalSpeed,
//                            -currentGamepad1.left_stick_x * RobotConstants.strafeSpeed * finalSpeed
//                            ,-currentGamepad1.right_stick_x*Math.abs(currentGamepad1.right_stick_x) * RobotConstants.turnSpeed);
//
//                    if (currentGamepad1.start) {
//                        driveState.setValue("Corrective Turning");
//                        targetHeading = robot.getPoseEstimate().getHeading();
//                        driveStates = DriveStates.CorrectiveTurning;
//                    } else if (currentGamepad1.y) {
//                        driveState.setValue("Field Centric");
//                        targetHeading = robot.getPoseEstimate().getHeading();
//                        driveStates = DriveStates.FieldCentric;
//                    }
//                    break;
//                case CorrectiveTurning:
//                    robot.setMecanumDriveHeadingPriority(-currentGamepad1.left_stick_y * RobotConstants.driveSpeed * finalSpeed,
//                            -currentGamepad1.left_stick_x * RobotConstants.strafeSpeed * finalSpeed
//                            , headingComponent);
//
//                    if (currentGamepad1.back) {
//                        driveState.setValue("Basic Turning");
//                        driveStates = DriveStates.BasicTurning;
//                    } else if (currentGamepad1.y) {
//                        driveState.setValue("Field Centric");
//                        driveStates = DriveStates.FieldCentric;
//                    }
//                    break;
//                case FieldCentric:
//                    robot.setMecanumDriveFieldCentricHeadingPriority(-currentGamepad1.left_stick_y * RobotConstants.driveSpeed * finalSpeed,
//                            -currentGamepad1.left_stick_x * RobotConstants.strafeSpeed * finalSpeed
//                            , headingComponent, robot.getPoseEstimate().getHeading());
//
//                    if (currentGamepad1.back) {
//                        driveState.setValue("Basic Turning");
//                        driveStates = DriveStates.BasicTurning;
//                    } else if (currentGamepad1.start) {
//                        driveState.setValue("Corrective Turning");
//                        targetHeading = robot.getPoseEstimate().getHeading();
//                        driveStates = DriveStates.CorrectiveTurning;
//                    }
//                    break;
//            }
//
//            //Resetting heading for field-centric
//            if (currentGamepad1.b && !prevGamepad1.b) {
//                targetHeading = 0;
//                robot.setPoseEstimate(new Pose2d(robot.getPoseEstimate().getX(), robot.getPoseEstimate().getY(), 0));
//            }

            //Climb code
            switch (climb) {
                case HOLD:
                    if (currentGamepad1.dpad_down) {// && currentGamepad1.right_bumper
                        targetClimbPos = robot.climbMotor.getCurrentPosition() + 200;
                        robot.climbMotor.setTargetPosition((int) targetClimbPos);
                        climbTimer.reset();
                        climb = Climb.RELEASE;
                    }
                    break;
                case RELEASE:
                    if (climbTimer.milliseconds()>RobotConstants.climbReleaseDelay && !currentGamepad1.dpad_down) {
                        climb = Climb.WAIT;
                    }
                    break;
                case WAIT:
                    if (currentGamepad1.dpad_down) {
                        driveStates = DriveStates.BasicTurning;
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
            if (currentGamepad1.left_bumper && currentGamepad1.right_bumper) {
                robot.releaseDrone();
            }

            if (currentGamepad1.dpad_left) {
                robot.rightPixelServo.setPosition(RobotConstants.rightIn);
            } else if (currentGamepad1.dpad_right)
                robot.rightPixelServo.setPosition(RobotConstants.rightOut);

            //Driver 2 code
            //Code for dropping pixels out of outtake
            switch (drop) {
                case CLOSED:
                    if (currentGamepad1.left_bumper) {
                        robot.dropper(RobotConfig.Dropper.OPEN);
                        dropTimer.reset();
                        drop = Drop.OPEN;
                    } else if (currentGamepad1.right_bumper) {
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
            if (abs(currentGamepad1.left_stick_y)>.3) {
                robot.setTargetSlidePos(robot.getTargetSlidePos()-currentGamepad1.left_stick_y*slideTimer.seconds()*Math.abs(currentGamepad1.left_stick_y*slideTimer.seconds())*10);
            } else if (currentGamepad1.a) {
                robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
            } else if (currentGamepad1.x) {
                robot.setTargetSlidePos(RobotConfig.SlideHeight.LOW);
            } else if (currentGamepad1.b) {
                robot.setTargetSlidePos(RobotConfig.SlideHeight.MEDIUM);
            } else if (currentGamepad1.y) {
                robot.setTargetSlidePos(RobotConfig.SlideHeight.HIGH);
            }

            slideTimer.reset();

            //Intake code
            switch (spikemark) {
                case GUIDE:
                    if (currentGamepad1.left_trigger > .1) {
                        robot.stackArm(RobotConfig.StackArm.OUT);
                        spikemark = SpikeMark.OUT;
                    }
                    break;
                case OUT:
                    if (currentGamepad1.left_trigger > .9) {
                        robot.stackHold(true);
                        spikemark = SpikeMark.HOLD;
                    } else if (currentGamepad1.left_trigger < .1) {
                        robot.stackArm(RobotConfig.StackArm.GUIDE);
                        spikemark = SpikeMark.GUIDE;
                    }
                    break;
                case HOLD:
                    if (currentGamepad1.right_trigger > .1) {
                        robot.stackArm(RobotConfig.StackArm.IN);
                        spikemark = SpikeMark.IN;
                    } else if (currentGamepad1.left_trigger < .9) {
                        robot.stackHold(false);
                        spikemark = SpikeMark.OUT;
                    }
                    break;
                case IN:
                    if (currentGamepad1.right_trigger < .1) {
                        robot.stackArm(RobotConfig.StackArm.OUT);
                        spikemark = SpikeMark.HOLD;
                    }
                    break;
            }

            switch (intake) {
                case STOPPED:
                    if (currentGamepad1.right_stick_y>.3) {
                        robot.intakeMotor.setPower(RobotConstants.intakeSpeed);
                        intake = Spin.SPIN_IN;
                    } else if (currentGamepad1.right_stick_y<-.3) {
                        robot.intakeMotor.setPower(-RobotConstants.intakeSpeed);
                        intake = Spin.SPIN_OUT;
                    }
                    break;
                case SPIN_IN:
                    if (currentGamepad1.right_stick_y<.3) {
                        robot.intakeMotor.setPower(0);
                        intake = Spin.STOPPED;
                    }
                    break;
                case SPIN_OUT:
                    if (currentGamepad1.right_stick_y>-.3) {
                        robot.intakeMotor.setPower(0);
                        intake = Spin.STOPPED;
                    }
                    break;
            }

            switch (transfer) {
                case STOPPED:
                    if (currentGamepad1.right_stick_x>.3) {
                        robot.transferMotor.setPower(RobotConstants.transferSpeed);
                        transfer = Spin.SPIN_IN;
                    } else if (currentGamepad1.right_stick_x<-.3) {
                        robot.transferMotor.setPower(-RobotConstants.transferSpeed);
                        transfer = Spin.SPIN_OUT;
                    }
                    break;
                case SPIN_IN:
                    if (currentGamepad1.right_stick_x<.3) {
                        robot.transferMotor.setPower(0);
                        transfer = Spin.STOPPED;
                    }
                    break;
                case SPIN_OUT:
                    if (currentGamepad1.right_stick_x>-.3) {
                        robot.transferMotor.setPower(0);
                        transfer = Spin.STOPPED;
                    }
                    break;
            }

            robot.update();
            robotPose.setValue(RobotMethods.updateRobotPosition(robot.getPoseEstimate()));
            odomPositions.setValue(robot.localizer.getWheelPositions());
//            imuAngle.setValue(robot.getRawExternalHeading()*180/Math.PI);

            prevGamepad1.copy(currentGamepad1);
            prevGamepad2.copy(currentGamepad2);

            loopTime.setValue(String.format("%,3.2f ms", loopTimer.milliseconds()));
            loopTimer.reset();
            telemetry.update();
        }


    }

}
