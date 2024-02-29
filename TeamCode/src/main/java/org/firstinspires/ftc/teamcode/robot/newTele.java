package org.firstinspires.ftc.teamcode.robot;

import static java.lang.Math.abs;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.RobotConfig;


@TeleOp
public class newTele extends LinearOpMode {

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

//    enum SpikeMark {
//        GUIDE,
//        OUT,
//        HOLD,
//        IN
//    }
//    SpikeMark spikemark = SpikeMark.GUIDE;
    enum ArmToggle {
    GUIDE,
        OUT
    }
    ArmToggle armToggle = ArmToggle.GUIDE;

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
        Telemetry.Item currentIMU = telemetry.addData("IMU", "");
        Telemetry.Item robotPose = telemetry.addData("Robot pose", RobotMethods.updateRobotPosition(robot.getPoseEstimate()));
        Telemetry.Item odomPositions = telemetry.addData("Encoder Positoions", "");
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

        status.setValue("Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        robot.stackHold(false);
        robot.stackArm(RobotConfig.StackArm.GUIDE);
        robot.dropper(RobotConfig.Dropper.CLOSED);

        status.setValue("Running");

        loopTimer.reset();
        slideTimer.reset();

        while (!isStopRequested()) {
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            double finalSpeed = RobotConstants.speedMultiplier * (1 + (currentGamepad1.right_trigger - (currentGamepad1.left_trigger)*.4) / 1.2);

            if (abs(currentGamepad1.right_stick_x)>.05) {
                targetHeading = RobotMethods.fastInRangeRad( targetHeading-currentGamepad1.right_stick_x* RobotConstants.turnSpeed * driveTrainTimer.seconds()*3.5*(1-currentGamepad1.left_trigger*.6)
                );
            }
            driveTrainTimer.reset();
            double vel = 0;
            if (robot.getPoseVelocity() != null) {
                vel = robot.getPoseVelocity().getHeading();
            }
            double headingDiff = RobotMethods.fastAngleDifferenceRad(targetHeading,robot.getPoseEstimate().getHeading());
            double absHeadingDiff = abs(headingDiff);

            double headingComponent = 0;

            if (absHeadingDiff>Math.toRadians(25)) {
                headingComponent = Math.signum(headingDiff)/2;
            } else if (absHeadingDiff>Math.toRadians(8)) {
                headingComponent = headingDiff*1.0;// + Math.signum(headingDiff)/15
            } else if (absHeadingDiff>Math.toRadians(3)) {
                headingComponent = headingDiff*.6 + Math.signum(headingDiff)/21 + .012*vel;
            } else if (absHeadingDiff>Math.toRadians(1)) {
                headingComponent = headingDiff*.4 + Math.signum(headingDiff)/26 + .008*vel;
            }

            turnVals.setValue(String.format("%,3.2f Velocity %,3.2f", headingComponent, vel));

            currentIMU.setValue(robot.getCurrentIMU().toString());

            switch (driveStates) {
                case BasicTurning:
                    robot.setMecanumDrive(-currentGamepad1.left_stick_y * RobotConstants.driveSpeed * finalSpeed,
                            -currentGamepad1.left_stick_x * RobotConstants.strafeSpeed * finalSpeed
                            ,-currentGamepad1.right_stick_x * RobotConstants.turnSpeed);// * RobotConstants.turnSpeed)

                    if (currentGamepad1.start) {
                        driveState.setValue("Corrective Turning");
                        targetHeading = robot.getPoseEstimate().getHeading();
                        driveStates = DriveStates.CorrectiveTurning;
                    } else if (currentGamepad1.y) {
                        driveState.setValue("Field Centric");
                        targetHeading = robot.getPoseEstimate().getHeading();
                        driveStates = DriveStates.FieldCentric;
                    }
                    break;
                case CorrectiveTurning:
                    robot.setMecanumDriveHeadingPriority(-currentGamepad1.left_stick_y * RobotConstants.driveSpeed * finalSpeed,
                            -currentGamepad1.left_stick_x * RobotConstants.strafeSpeed * finalSpeed
                            , headingComponent);

                    if (currentGamepad1.back) {
                        driveState.setValue("Basic Turning");
                        driveStates = DriveStates.BasicTurning;
                    } else if (currentGamepad1.y) {
                        driveState.setValue("Field Centric");
                        driveStates = DriveStates.FieldCentric;
                    }
                    break;
                case FieldCentric:
                    robot.setMecanumDriveFieldCentricHeadingPriority(-currentGamepad1.left_stick_y * RobotConstants.driveSpeed * finalSpeed,
                            -currentGamepad1.left_stick_x * RobotConstants.strafeSpeed * finalSpeed
                            , headingComponent, robot.getPoseEstimate().getHeading());

                    if (currentGamepad1.back) {
                        driveState.setValue("Basic Turning");
                        driveStates = DriveStates.BasicTurning;
                    } else if (currentGamepad1.start) {
                        driveState.setValue("Corrective Turning");
                        targetHeading = robot.getPoseEstimate().getHeading();
                        driveStates = DriveStates.CorrectiveTurning;
                    }
                    break;
            }

            //Resetting heading for field-centric
            if (currentGamepad1.b && !prevGamepad1.b) {
                targetHeading = 0;
                robot.setPoseEstimate(new Pose2d(robot.getPoseEstimate().getX(), robot.getPoseEstimate().getY(), 0));
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
            if (currentGamepad2.a) {
                robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
            } else if (currentGamepad2.x) {
                robot.setTargetSlidePos(RobotConfig.SlideHeight.LOW);
            } else if (currentGamepad2.b) {
                robot.setTargetSlidePos(RobotConfig.SlideHeight.MEDIUM);
            } else if (currentGamepad2.y) {
                robot.setTargetSlidePos(RobotConfig.SlideHeight.HIGH);
            } else if (abs(currentGamepad2.left_stick_y)>.1) {
                robot.setTargetSlidePos(robot.getTargetSlidePos()-currentGamepad2.left_stick_y*slideTimer.seconds()*Math.abs(currentGamepad2.left_stick_y*slideTimer.seconds())*300);
            }

            slideTimer.reset();

            //Intake code
            if (currentGamepad2.left_trigger>.5 && prevGamepad2.left_trigger<=.5) {
                robot.grabFromStack(2);
                armToggle = ArmToggle.OUT;
            } else if (currentGamepad2.right_trigger>.5 && prevGamepad2.right_trigger<=.5) {
                robot.grabFromStack(1);
                armToggle = ArmToggle.OUT;
            }

            switch (armToggle) {
                case GUIDE:
                    if (currentGamepad2.back && !prevGamepad2.back) {
                        robot.stackArm(RobotConfig.StackArm.OUT);
                        armToggle = ArmToggle.OUT;
                    }
                    break;
                case OUT:
                    if (currentGamepad2.back && !prevGamepad2.back) {
                        robot.stackArm(RobotConfig.StackArm.GUIDE);
                        armToggle = ArmToggle.GUIDE;
                    }
                    break;
            }


            switch (intake) {
                case STOPPED:
                    if (currentGamepad2.dpad_down) {// || currentGamepad2.left_trigger >.1
                        robot.intakeMotor.setPower(RobotConstants.intakeSpeed);
                        intake = Spin.SPIN_IN;
                    } else if (currentGamepad2.dpad_up) {
                        robot.intakeMotor.setPower(-RobotConstants.intakeSpeed);
                        intake = Spin.SPIN_OUT;
                    }
                    break;
                case SPIN_IN:
                    if (!currentGamepad2.dpad_down) {// || currentGamepad2.left_trigger <.1
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
                    if (currentGamepad2.dpad_right) {// || currentGamepad2.left_trigger >.1
                        robot.transferMotor.setPower(RobotConstants.transferSpeed);
                        transfer = Spin.SPIN_IN;
                    } else if (currentGamepad2.dpad_left) {
                        robot.transferMotor.setPower(-RobotConstants.transferSpeed);
                        transfer = Spin.SPIN_OUT;
                    }
                    break;
                case SPIN_IN:
                    if (!currentGamepad2.dpad_right) {// || currentGamepad2.left_trigger <.1
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
            robotPose.setValue(RobotMethods.updateRobotPosition(robot.getPoseEstimate()));
            odomPositions.setValue(robot.localizer.getWheelPositions());

            prevGamepad1.copy(currentGamepad1);
            prevGamepad2.copy(currentGamepad2);

            loopTime.setValue(String.format("%,3.2f ms", loopTimer.milliseconds()));
            loopTimer.reset();
            telemetry.update();
        }


    }

}
