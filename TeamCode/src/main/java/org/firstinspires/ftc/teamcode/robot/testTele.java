package org.firstinspires.ftc.teamcode.robot;


import static java.lang.Math.abs;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@TeleOp()
public class testTele extends LinearOpMode {

    //Establish variables
    double maxSpeed = 1;

    enum DriveStates {
        CorrectiveTurning,
        FieldCentric,
        BasicTurning
    }

    //target positions
    double targetX, targetY;

    double lastTurn = 0;

    //decides if robot uses field centric or robot centric driving

    DriveStates driveStates = DriveStates.CorrectiveTurning;

    enum Drop {
        OPEN,
        DOUBLE_OPEN,
        CLOSED,
        RESET
    }
    Drop drop = Drop.CLOSED;

    enum Climb {
        HOLD,
        RELEASE,
        WAIT,
        STOPPED,
//        SPIN_IN,
//        SPIN_OUT
    }
    Climb climb = Climb.HOLD;

    enum Spin {
        STOPPED,
        SPIN_IN,
        SPIN_OUT
    }
    Spin transfer = Spin.STOPPED;
    Spin intake = Spin.STOPPED;

    enum SpikeMark {
        GUIDE,
        OUT,
        HOLD,
        IN
    }
    SpikeMark spikemark = SpikeMark.GUIDE;

    enum Slide {
        BOTTOM,
        LOW,
        MIDDLE,
        TOP
    }
    Slide slidePos = Slide.BOTTOM;

    int targetPos = RobotConstants.slideBottom;


    double slideI = 0.0;

    int targetClimbPos;

    //Used to store data from april tags
    AprilTagDetection frontCamAprilTags = null;

    //Used to tell if camera has detected april tags
    boolean tagDetection;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(false);
        //Updating Status
        Telemetry.Item status = telemetry.addData("Status", "Initializing");
        telemetry.update();

        //Init code
        HwMap robot = new HwMap();
        robot.init(hardwareMap);
        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);


        ElapsedTime dropTimer = new ElapsedTime();

        ElapsedTime climbTimer = new ElapsedTime();

        ElapsedTime slideTimer = new ElapsedTime();

        ElapsedTime loopTimer = new ElapsedTime();

        ElapsedTime driveTrainTimer = new ElapsedTime();

        ElapsedTime intakeHeightDelayTimer = new ElapsedTime();

        ElapsedTime spikeTimer = new ElapsedTime();


//        ElapsedTime slideTimer = new ElapsedTime();

//        Runnable updateColorSensor = () -> outtakePixel.set(robot.outtakeColorSensor.red() > RobotConstants.outtakeValue);

//        Runnable updateDistanceSenor = () -> intakePixel.set(robot.intakeDistanceSensor.getDistance(DistanceUnit.CM)< RobotConstants.intakeValue);

        //Getting last pose
        driveTrain.setPoseEstimate(PassData.currentPose);


        double targetHeading = driveTrain.getPoseEstimate().getHeading();

        Telemetry.Item driveState = telemetry.addData("Drive Mode","Corrective Turning");

        Telemetry.Item turnValues = telemetry.addData("Turn vals", "");
        //Adding roadrunner pose to telemetry
        Telemetry.Item robotPose = telemetry.addData("Robot pose:", RobotMethods.updateRobotPosition(driveTrain.getPoseEstimate()));

        //Adding odom pod encoders to telemetry
        Telemetry.Item odom = telemetry.addData("Encoder Positions:", StandardTrackingWheelLocalizer.getEncoderVals());

//        Telemetry.Item aprilTagPosEstimate = telemetry.addData("April-tag Estimated Pos:", "");
//
//        Telemetry.Item aprilTagSolveTime = telemetry.addData("Solve Time:", "");

        Telemetry.Item slideData = telemetry.addData("Slide Data:", "Encoder Val:" + robot.liftEncoder.getCurrentPosition() + " Target Val:" + targetPos);

//        Telemetry.Item intakeHeight = telemetry.addData("Intake Height", "1");

        Telemetry.Item loopTime = telemetry.addData("Loop Time", "0ms");
        //Set starting positions
        robot.dropServo.setPosition(RobotConstants.dropClosed);


        robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide + RobotConstants.rightSpikeOffset);
        robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide);
        robot.spikeMarkHoldServo.setPosition(RobotConstants.holdServoUp);



        //Updating Status
        status.setValue("Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        status.setValue("Running");

        while (opModeIsActive() && !isStopRequested()) {
            loopTimer.reset();

            //Getting robots estimated position
            Pose2d myPose = driveTrain.getPoseEstimate();
            //Setting telemetry to display robots position
            robotPose.setValue(RobotMethods.updateRobotPosition(myPose));
            odom.setValue(StandardTrackingWheelLocalizer.getEncoderVals());

            //Driver 1 code
            //Front triggers being used to speedup or slowdown robots driving
            double finalSpeed = RobotConstants.speedMultiplier * (1 + (gamepad1.right_trigger - (gamepad1.left_trigger)*.4) / 1.2);

            double turnVelocity = (myPose.getHeading()-lastTurn)/driveTrainTimer.seconds();
            lastTurn = myPose.getHeading();
            //Using last turn here is the same as getting the current heading
            if (abs(gamepad1.right_stick_x)>.1) {
                targetHeading -= gamepad1.right_stick_x * RobotConstants.turnSpeed * driveTrainTimer.seconds()*4*finalSpeed;
// * (1-.7*(gamepad1.left_stick_y+gamepad1.left_stick_x))
                //gamepad1.right_stick_x * RobotConstants.turnSpeed * driveTrainTimer.seconds()*4;
                //Keeping range inside of 2pi
                if (targetHeading>=2*Math.PI) {
                    targetHeading -= 2*Math.PI;
                } else if (targetHeading<0) {
                    targetHeading += 2*Math.PI;
                }
            }

            double turnDegrees = (targetHeading-lastTurn);

            if (turnDegrees>Math.PI) {
                turnDegrees -= 2*Math.PI;
            } else if (turnDegrees < -Math.PI) {
                turnDegrees += 2*Math.PI;
            }

            double headingComponent = Range.clip(turnDegrees*2, -1, 1)-turnVelocity*.18;

            driveTrainTimer.reset();

            turnValues.setValue(String.format("Target heading: %,3.2f Degrees Off: %,3.2f Heading Component: %,3.2f", targetHeading, turnDegrees, headingComponent));


            switch (driveStates) {
                case CorrectiveTurning:
                    RobotMethods.setMecanumDriveHeadingPriority(-gamepad1.left_stick_y * RobotConstants.driveSpeed * finalSpeed,
                            -gamepad1.left_stick_x * RobotConstants.strafeSpeed * finalSpeed
                            , headingComponent, driveTrain);

                    if (gamepad1.back) {
                        driveState.setValue("Basic Turning");
                        driveStates = DriveStates.BasicTurning;
                    } else if (gamepad1.y) {
                        driveState.setValue("Field Centric");
                        driveStates = DriveStates.FieldCentric;
                    }
                    break;
                case FieldCentric:
                    RobotMethods.setMecanumDriveFieldCentricHeadingPriority(-gamepad1.left_stick_y * RobotConstants.driveSpeed * finalSpeed,
                            -gamepad1.left_stick_x * RobotConstants.strafeSpeed * finalSpeed
                            , headingComponent, lastTurn, driveTrain);

                    if (gamepad1.b) {
                        targetHeading = 0;
                        driveTrain.setPoseEstimate(new Pose2d(myPose.getX(), myPose.getY(), 0));
                    }

                    if (gamepad1.back) {
                        driveState.setValue("Basic Turning");
                        driveStates = DriveStates.BasicTurning;
                    } else if (gamepad1.start) {
                        driveState.setValue("Corrective Turning");
                        targetHeading = myPose.getHeading();
                        driveStates = DriveStates.CorrectiveTurning;
                    }
                    break;
                case BasicTurning:
                    RobotMethods.setMecanumDrive(-gamepad1.left_stick_y * RobotConstants.driveSpeed * finalSpeed,
                            -gamepad1.left_stick_x * RobotConstants.strafeSpeed * finalSpeed
                            , -gamepad1.right_stick_x * RobotConstants.turnSpeed, driveTrain);

                    if (gamepad1.start) {
                        driveState.setValue("Corrective Turning");
                        targetHeading = myPose.getHeading();
                        driveStates = DriveStates.CorrectiveTurning;
                    } else if (gamepad1.y) {
                        driveState.setValue("Field Centric");
                        targetHeading = myPose.getHeading();
                        driveStates = DriveStates.FieldCentric;
                    }
                    break;
            }

            //Climb code
            switch (climb) {
                case HOLD:
                    if (gamepad1.dpad_down && gamepad1.right_bumper) {
                        targetClimbPos = robot.climbMotor.getCurrentPosition() + 200;
                        robot.climbMotor.setTargetPosition(targetClimbPos);
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
                    if (gamepad1.dpad_down) {
                        driveStates = DriveStates.BasicTurning;
                        climbTimer.reset();
                        climb = Climb.STOPPED;
                    }
                    break;
                case STOPPED:
                    if (gamepad1.dpad_down) {
                        targetClimbPos +=  2.5*climbTimer.milliseconds();
                        robot.climbMotor.setTargetPosition(targetClimbPos);
                    } else if (gamepad1.dpad_up) {
                        targetClimbPos -=  2.5*climbTimer.milliseconds();
                        robot.climbMotor.setTargetPosition(targetClimbPos);
                    }
                    climbTimer.reset();
                    break;
            }

            //Drone Launch
            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                robot.droneServo.setPosition(RobotConstants.droneRelease);
            }


            //Driver 2 code
            //Code for dropping pixels out of outtake
            switch (drop) {
                case CLOSED:
                    if (gamepad2.left_bumper) {
                        robot.dropServo.setPosition(RobotConstants.dropOpen);
                        dropTimer.reset();
                        drop = Drop.OPEN;
                    } else if (gamepad2.right_bumper) {
                        robot.dropServo.setPosition(RobotConstants.dropPartial);
                        dropTimer.reset();
                        drop = Drop.DOUBLE_OPEN;
                    }
                    break;
                case OPEN:
                    if (dropTimer.seconds() > RobotConstants.dropTime) {
                        robot.dropServo.setPosition(RobotConstants.dropClosed);
                        dropTimer.reset();
                        drop = Drop.RESET;
                    }
                    break;
                case DOUBLE_OPEN:
                    if (dropTimer.seconds() > RobotConstants.doubleDropTime) {
                        robot.dropServo.setPosition(RobotConstants.dropClosed);
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

            //Code to change target position of slides
            if (abs(gamepad2.left_stick_y)>.1) {
                targetPos -= 14 * gamepad2.left_stick_y*slideTimer.milliseconds();
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

            //Slide PID control
            double slideVelo = robot.liftEncoder.getCorrectedVelocity();
            int slideCurPos = robot.liftEncoder.getCurrentPosition();

            double distRemain = targetPos - slideCurPos;

            slideI += distRemain * RobotConstants.slidePIDVals.i;

            double slidePower = (distRemain * RobotConstants.slidePIDVals.p) + slideI + .16 + (slideVelo * RobotConstants.slidePIDVals.d);

            robot.slideMotor.setPower(slidePower);

            slideData.setValue( String.format("Encoder val: %d Target Val: %d Slide Power: %,3.2f", slideCurPos, targetPos, slidePower));


            switch (intake) {
                case STOPPED:
                    if (gamepad2.dpad_down || gamepad2.left_trigger >.1) {
                        robot.intakeMotor.setPower(RobotConstants.intakeSpeed);
                        intake = Spin.SPIN_IN;
                    } else if (gamepad2.dpad_up) {
                        robot.intakeMotor.setPower(-RobotConstants.intakeSpeed);
                        intake = Spin.SPIN_OUT;
                    }
                    break;
                case SPIN_IN:
                    if (!gamepad2.dpad_down || gamepad2.left_trigger <.1) {
                        robot.intakeMotor.setPower(0);
                        intake = Spin.STOPPED;
                    }
                    break;
                case SPIN_OUT:
                    if (!gamepad2.dpad_up) {
                        robot.intakeMotor.setPower(0);
                        intake = Spin.STOPPED;
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
                    } else if (gamepad2.dpad_left) {
                        robot.transferMotor.setPower(-RobotConstants.transferSpeed);
                        transfer = Spin.SPIN_OUT;
                    }
                    break;
                case SPIN_IN:
                    if (!gamepad2.dpad_right || gamepad2.left_trigger <.1) {
                        robot.transferMotor.setPower(0);
                        transfer = Spin.STOPPED;
                    }
                    break;
                case SPIN_OUT:
                    if (!gamepad2.dpad_left) {
                        robot.transferMotor.setPower(0);
                        transfer = Spin.STOPPED;
                    }
                    break;
            }

            //Updating for roadrunner
            driveTrain.update();

            loopTime.setValue(String.format("%,3.2f ms", loopTimer.milliseconds()));

            //Updating telemetry
            telemetry.update();
        }
        robot.slideMotor.setPower(0.0);

    }


}
//Code that i got rid of because it relied on i2c and i2c sensors brought loop times down to 4ish loops per second
//i2c code is at end of loop to give threads time to finish
//Adds to the robotPixelCount when pixel is detected
//            switch (intakeCount) {
//                case CLEAR:
//                    if (intakePixel.get()) {
//                        robotPixelCount += 1;
//                        intakeCount = IntakeCount.PIXEL;
//                    }
//                    break;
//                case PIXEL:
//                    if (!outtakePixel.get()) {
//                        intakeCount = IntakeCount.CLEAR;
//                    }
//                    break;
//            }
//
//            //Sets outtakePixelCount to 1 if pixel is temporarily detected and 2 if pixel remains
//            switch (outtakeCount) {
//                case CLEAR:
//                    if (outtakePixel.get()) {
//                        outtakePixelCount = 1;
//                        outtakeTimer.reset();
//                        outtakeCount = OuttakeCount.WAIT;
//                    }
//                    break;
//                case WAIT:
//                    if (outtakeTimer.milliseconds()>RobotConstants.outtakeCountDelay) {
//                        outtakeCount = OuttakeCount.RECHECK;
//                    }
//                    break;
//                case RECHECK:
//                    if (outtakePixel.get()) {
//                        outtakePixelCount = 2;
//                        //Sets to blocked so switch doesn't keep looping, OuttakeCount is set back to clear in the drop switch statement when a pixel is dropped
//                        outtakeCount = OuttakeCount.BLOCKED;
//                    } else {
//                        outtakeCount = OuttakeCount.CLEAR;
//                    }
//                    break;
//            }

//Automatically runs the transfer if pixels are in the robot but not in the outtake
//            switch (smartTransfer) {
//                case STOPPED:
//                    if (gamepad2.dpad_right) {
//                        robot.transferMotor.setPower(RobotConstants.transferSpeed);
//                        smartTransfer = SmartTransfer.MANUAL_SPIN_IN;
//                    } else if (gamepad2.dpad_left) {
//                        robot.transferMotor.setPower(-RobotConstants.transferSpeed);
//                        smartTransfer = SmartTransfer.MANUAL_SPIN_OUT;
//                    } else if (!(outtakePixelCount==2) && outtakePixelCount<robotPixelCount && useSmartTransfer) {
//                        robot.transferMotor.setPower(RobotConstants.transferSpeed);
//                        smartTransfer = SmartTransfer.AUTO_SPIN;
//                    }
//                    break;
//                case MANUAL_SPIN_IN:
//                    if (!gamepad2.dpad_right) {
//                        robot.transferMotor.setPower(0);
//                        smartTransfer = SmartTransfer.STOPPED;
//                    }
//                    break;
//                case MANUAL_SPIN_OUT:
//                    if (!gamepad2.dpad_left) {
//                        robot.transferMotor.setPower(0);
//                        smartTransfer = SmartTransfer.STOPPED;
//                    }
//                    break;
//                case AUTO_SPIN:
//                    if (outtakePixelCount==2 || outtakePixelCount>=robotPixelCount && !gamepad2.dpad_right && !gamepad2.dpad_left) {
//                        robot.transferMotor.setPower(0);
//                        smartTransfer = SmartTransfer.STOPPED;
//                    }
//                    break;
//            }

//Temporarily reverses intake shortly after robot detects that it has 2 pixels,
//            switch (smartIntake) {
//                case STOPPED:
//                    if(gamepad2.dpad_down) {
//                        robot.intakeMotor.setPower(RobotConstants.intakeSpeed);
//                        smartIntake = SmartIntake.MANUAL_SPIN_IN;
//                    } else if (gamepad2.dpad_up) {
//                        robot.intakeMotor.setPower(-RobotConstants.intakeSpeed);
//                        smartIntake = SmartIntake.MANUAL_SPIN_OUT;
//                    }
//                    break;
//                case MANUAL_SPIN_IN:
//                    if (!gamepad2.dpad_down) {
//                        robot.intakeMotor.setPower(0);
//                        smartIntake = SmartIntake.STOPPED;
//                    } else if (robotPixelCount>=2 && useSmartIntake) {
//                        intakeTimer.reset();
//                        smartIntake = SmartIntake.WAIT;
//                    }
//                        break;
//                case WAIT:
//                    if (intakeTimer.milliseconds()>RobotConstants.intakeReverseDelay) {
//                        robot.intakeMotor.setPower(-RobotConstants.intakeSpeed);
//                        intakeTimer.reset();
//                        smartIntake = SmartIntake.KICK_OUT;
//                    }
//                    break;
//                case KICK_OUT:
//                    if (intakeTimer.milliseconds()>RobotConstants.intakeReverseTime) {
//                        robot.intakeMotor.setPower(0);
//                        //Uses wait for release so that robot wont start in-taking until the button is released and unpressed; otherwise robot will keep reversing until the button is released
//                        smartIntake = SmartIntake.WAIT_FOR_RELEASE;
//                    }
//                    break;
//                case WAIT_FOR_RELEASE:
//                    if (!gamepad2.dpad_down) {
//                        smartIntake = SmartIntake.STOPPED;
//                    }
//                case MANUAL_SPIN_OUT:
//                    if (!gamepad2.dpad_up) {
//                        robot.intakeMotor.setPower(0);
//                        smartIntake = SmartIntake.STOPPED;
//                    }
//            }

//switch (slidePos) {
//                    case BOTTOM:
//                        if (gamepad2.x) {
//                            //                        roboMethods.setTargetPos(RobotConstants.slideBottom, RobotConstants.slideLow);
//                            slidePos = Slide.LOW;
//                            targetPos = RobotConstants.slideLow;
//                        } else if (gamepad2.b) {
//                            //                        roboMethods.setTargetPos(RobotConstants.slideBottom, RobotConstants.slideMiddle);
//                            slidePos = Slide.MIDDLE;
//                            targetPos = RobotConstants.slideMiddle;
//                        } else if (gamepad2.y) {
//                            //                        roboMethods.setTargetPos(RobotConstants.slideBottom, RobotConstants.slideTop);
//                            slidePos = Slide.TOP;
//                            targetPos = RobotConstants.slideTop;
//                        }
//                        break;
//                    case LOW:
//                        if (gamepad2.a) {
//                            //                        roboMethods.setTargetPos(RobotConstants.slideLow, RobotConstants.slideBottom);
//                            slidePos = Slide.BOTTOM;
//                            targetPos = RobotConstants.slideBottom;
//                        } else if (gamepad2.b) {
//                            //                        roboMethods.setTargetPos(RobotConstants.slideLow, RobotConstants.slideMiddle);
//                            slidePos = Slide.MIDDLE;
//                            targetPos = RobotConstants.slideMiddle;
//                        } else if (gamepad2.y) {
//                            //                        roboMethods.setTargetPos(RobotConstants.slideLow, RobotConstants.slideTop);
//                            slidePos = Slide.TOP;
//                            targetPos = RobotConstants.slideTop;
//
//                        }
//                        break;
//                    case MIDDLE:
//                        if (gamepad2.a) {
//                            //                        roboMethods.setTargetPos(RobotConstants.slideMiddle, RobotConstants.slideBottom);
//                            slidePos = Slide.BOTTOM;
//                            targetPos = RobotConstants.slideBottom;
//                        } else if (gamepad2.x) {
//                            //                        roboMethods.setTargetPos(RobotConstants.slideMiddle, RobotConstants.slideLow);
//                            slidePos = Slide.LOW;
//                            targetPos = RobotConstants.slideLow;
//                        } else if (gamepad2.y) {
//                            //                        roboMethods.setTargetPos(RobotConstants.slideMiddle, RobotConstants.slideTop);
//                            slidePos = Slide.TOP;
//                            targetPos = RobotConstants.slideTop;
//                        }
//                        break;
//                    case TOP:
//                        if (gamepad2.a) {
//                            //                        roboMethods.setTargetPos(RobotConstants.slideTop, RobotConstants.slideBottom);
//                            slidePos = Slide.BOTTOM;
//                            targetPos = RobotConstants.slideBottom;
//                        } else if (gamepad2.b) {
//                            //                        roboMethods.setTargetPos(RobotConstants.slideTop, RobotConstants.slideMiddle);
//                            slidePos = Slide.MIDDLE;
//                            targetPos = RobotConstants.slideMiddle;
//                        } else if (gamepad2.x) {
//                            //                        roboMethods.setTargetPos(RobotConstants.slideTop, RobotConstants.slideLow);
//                            slidePos = Slide.LOW;
//                            targetPos = RobotConstants.slideLow;
//                        }
//                        break;
//                }
//            }


// Drivestates old code
//case GAMEPAD:
//                    //Calculating velocity
//
//                    //Setting drive speeds for the robot
//                    RobotMethods.setMecanumDriveHeadingPriority(-gamepad1.left_stick_y * RobotConstants.driveSpeed * finalSpeed,
//                            -gamepad1.left_stick_x * RobotConstants.strafeSpeed * finalSpeed
//                            , headingComponent, driveTrain);
////Range.clip(turnPower*2-turnVelocity*.16, -4, 4)
//
//                    //Aligns robot to backboard if april tags have a detection
////                    if (gamepad1.left_bumper && tagDetection) {
////                        driveState.setValue(driveModeName + " Drive State: Re-localize");
////                        driveStates = DriveStates.RELOCALIZE;
////                    }
//                    break;
//                case GAMEPAD_FIELDCENTRIC:
//                    //Setting drive speeds for the robot
//                    RobotMethods.setMecanumDriveFieldCentric(-gamepad1.left_stick_y * RobotConstants.driveSpeed * finalSpeed,
//                            -gamepad1.left_stick_x * RobotConstants.strafeSpeed * finalSpeed,
//                            -gamepad1.right_stick_x * RobotConstants.turnSpeed * finalSpeed,
//                            maxSpeed, driveTrain.getPoseEstimate().getHeading(), driveTrain);
//
//                    //Aligns robot to backboard if april tags have a detection
//                    if (gamepad1.left_bumper && tagDetection) {
//                        driveState.setValue(driveModeName + " Drive State: Re-localize");
//                        driveStates = DriveStates.RELOCALIZE;
//                    }
//                    break;
//                case RELOCALIZE:
//                    //Setting the pose of the robot to pose detected by april tags
//                    driveTrain.setPoseEstimate(new Pose2d(frontCamAprilTags.ftcPose.x, frontCamAprilTags.ftcPose.y, frontCamAprilTags.ftcPose.yaw));
//
//                    //Setting target X of the robot
//                    targetX = RobotConstants.backDropAlignX;
//
//                    //Deciding which backboard to align to based on the robots pose
//                    if (frontCamAprilTags.ftcPose.y>0) {
//                        targetY = RobotConstants.backDropLeftY;
//                    } else {
//                        targetY = RobotConstants.backDropRightY;
//                    }
//                    driveState.setValue(driveModeName + " Drive State: Align");
//                    driveStates = DriveStates.ALIGN;
//                    break;
//                case ALIGN:
//                    //sets state back to default if drive1 exits the mode
//                    if (!gamepad1.left_bumper) {
//                        driveState.setValue(driveModeName + " Drive State: Drive");
//                        driveStates = driveMode;
//                    }
//
//                    RobotMethods.goToPoint(targetX, targetY, 0, driveTrain);
//                    if (abs(targetX-driveTrain.getPoseEstimate().getX())<.3 && abs(targetY-driveTrain.getPoseEstimate().getY())<.3) {
//                        driveState.setValue(driveModeName + " Drive State: Hold");
//                        driveStates = DriveStates.HOLD;
//                    }
//                    break;
//                case HOLD:
//                    //sets state back to default if drive1 exits the mode
//                    if (!gamepad1.left_bumper) {
//                        driveState.setValue(driveModeName + " Drive State: Drive");
//                        driveStates = driveMode;
//                    }
//
//                    RobotMethods.goToLineY(targetX, gamepad1.left_stick_x * RobotConstants.strafeSpeed * finalSpeed, 0, driveTrain);
//                    break;
//                default:
//                    driveState.setValue(driveModeName + " Drive State: Drive");
//                    driveStates = driveMode;