package org.firstinspires.ftc.teamcode.robot;


import static java.lang.Math.abs;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.atomic.AtomicReference;


@TeleOp()
public class testTele extends LinearOpMode {

    //Establish variables
    double maxSpeed = 1;

    enum DriveStates {
        GAMEPAD,
        GAMEPAD_FIELDCENTRIC,
        RELOCALIZE,
        ALIGN,
        HOLD
    }

    //target positions
    double targetX, targetY;

    //decides if robot uses field centric or robot centric driving
    DriveStates driveMode = DriveStates.GAMEPAD;

    String driveModeName = "Robot-centric";

    DriveStates driveStates = driveMode;

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

//    enum IntakeCount {
//        CLEAR,
//        PIXEL
//    }
//    IntakeCount intakeCount = IntakeCount.CLEAR;
//    short robotPixelCount = 0;
//
//    enum OuttakeCount {
//        CLEAR,
//        WAIT,
//        RECHECK,
//        BLOCKED
//    }
//    OuttakeCount outtakeCount = OuttakeCount.CLEAR;
//
//    short outtakePixelCount = 0;
//
//    //Using AtomicReference bc variables are accessed by multiple threads
//    //True when color black is not sensed in outtake
//    AtomicReference<Boolean > outtakePixel = new AtomicReference<>(false);
//    //True when distance sensor reads less than certain value
//    AtomicReference<Boolean> intakePixel = new AtomicReference<>(false);
//
//    enum SmartTransfer {
//        STOPPED,
//        MANUAL_SPIN_IN,
//        MANUAL_SPIN_OUT,
//        AUTO_SPIN
//    }
//    SmartTransfer smartTransfer = SmartTransfer.STOPPED;
//    boolean useSmartTransfer = false;
//
//    enum SmartIntake {
//        STOPPED,
//        MANUAL_SPIN_IN,
//        WAIT,
//        KICK_OUT,
//        WAIT_FOR_RELEASE,
//        MANUAL_SPIN_OUT
//    }
//    SmartIntake smartIntake = SmartIntake.STOPPED;
//    boolean useSmartIntake = false;


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

        AprilTagProcessor frontAprilTagProcessor;
        VisionPortal frontVisionPortal;
        frontAprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();


        //Creates visionPortal with configured setting, passes the webcam and the aprilTag Processor
        frontVisionPortal = new VisionPortal.Builder()
                .addProcessor(frontAprilTagProcessor)
                .setCamera(robot.frontCamera)
                //sets camera resolution to 640 by 480 so that we can use a default calibration
                .setCameraResolution(new Size(640,480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();


        ElapsedTime dropTimer = new ElapsedTime();

        ElapsedTime climbTimer = new ElapsedTime();

        ElapsedTime slideTimer = new ElapsedTime();

        ElapsedTime loopTimer = new ElapsedTime();


//        ElapsedTime slideTimer = new ElapsedTime();

//        Runnable updateColorSensor = () -> outtakePixel.set(robot.outtakeColorSensor.red() > RobotConstants.outtakeValue);

//        Runnable updateDistanceSenor = () -> intakePixel.set(robot.intakeDistanceSensor.getDistance(DistanceUnit.CM)< RobotConstants.intakeValue);

        //Getting last pose
        driveTrain.setPoseEstimate(PassData.currentPose);

        Telemetry.Item driveState = telemetry.addData("Drive Mode", toString(),driveModeName + " Drive State: Drive");

        //Adding roadrunner pose to telemetry
        Telemetry.Item robotPose = telemetry.addData("Robot pose:", RobotMethods.updateRobotPosition(driveTrain.getPoseEstimate()));

        //Adding odom pod encoders to telemetry
        Telemetry.Item odom = telemetry.addData("Encoder Positions:", StandardTrackingWheelLocalizer.getEncoderVals());

        Telemetry.Item aprilTagPosEstimate = telemetry.addData("April-tag Estimated Pos:", "");

        Telemetry.Item slideData = telemetry.addData("Slide Data:", "Encoder Val:" + robot.liftEncoder.getCurrentPosition() + " Target Val:" + targetPos);

        Telemetry.Item loopTime = telemetry.addData("Loop Time", "0ms");
        //Set starting positions
        robot.dropServo.setPosition(RobotConstants.dropClosed);





        //Updating Status
        status.setValue("Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        status.setValue("Running");

        while (opModeIsActive() && !isStopRequested()) {
            loopTimer.reset();

            //Threads read i2c sensors and update variables async bc reading the sensors can be slow
//            updateColorSensor.run();
//            updateDistanceSenor.run();

            //Getting robots estimated position
            Pose2d myPose = driveTrain.getPoseEstimate();
            //Setting telemetry to display robots position
            robotPose.setValue(RobotMethods.updateRobotPosition(myPose));
            odom.setValue(StandardTrackingWheelLocalizer.getEncoderVals());

            //Getting aprilTag detections
//            if (frontAprilTagProcessor.getDetections().size() > 0) {
//                //Gets all the april tag data for the 1st detection
//                frontCamAprilTags = frontAprilTagProcessor.getDetections().get(0);
//                aprilTagPosEstimate.setValue(RobotMethods.updateRobotPosAprilTag(frontCamAprilTags));
//                tagDetection = true;
//            } else {
//                aprilTagPosEstimate.setValue("No tags detected");
//                tagDetection = false;
//            }


            //Driver 1 code
            //Front triggers being used to speedup or slowdown robots driving
            double finalSpeed = RobotConstants.speedMultiplier * (1 + (gamepad1.right_trigger - (gamepad1.left_trigger)*.6) / 1.2);

//            switch (driveMode) {
//                case GAMEPAD:
//                    if (gamepad1.start) {
//                        driveMode = DriveStates.GAMEPAD_FIELDCENTRIC;
//                        driveModeName = "Field-centric";
//                        if (driveStates == DriveStates.GAMEPAD) {
//                            driveState.setValue(driveModeName + " Drive State: Drive");
//                            driveStates = DriveStates.GAMEPAD_FIELDCENTRIC;
//                        }
//                    }
//                    break;
//                case GAMEPAD_FIELDCENTRIC:
//                    if (gamepad1.back) {
//                        driveMode = DriveStates.GAMEPAD;
//                        driveModeName = "Robot-centric";
//                        if (driveStates == DriveStates.GAMEPAD_FIELDCENTRIC) {
//                            driveState.setValue(driveModeName + " Drive State: Drive");
//                            driveStates = DriveStates.GAMEPAD;
//                        }
//                    }
//                    break;
//            }

            //Calculating and applying the powers for mecanum wheels
            //For field-centric driving replace below line with: robotMethods.setMecanumDriveFieldCentric(drive, strafe, turn, maxSpeed, myPose.getHeading(), driveTrain);
            switch (driveStates) {
                case GAMEPAD:
                    //Setting drive speeds for the robot
                    RobotMethods.setMecanumDrive(-gamepad1.left_stick_y * RobotConstants.driveSpeed * finalSpeed,
                            -gamepad1.left_stick_x * RobotConstants.strafeSpeed * finalSpeed,
                            -gamepad1.right_stick_x * RobotConstants.turnSpeed * finalSpeed,
                            maxSpeed, driveTrain);

                    //Aligns robot to backboard if april tags have a detection
//                    if (gamepad1.left_bumper && tagDetection) {
//                        driveState.setValue(driveModeName + " Drive State: Re-localize");
//                        driveStates = DriveStates.RELOCALIZE;
//                    }
                    break;
                case GAMEPAD_FIELDCENTRIC:
                    //Setting drive speeds for the robot
                    RobotMethods.setMecanumDriveFieldCentric(-gamepad1.left_stick_y * RobotConstants.driveSpeed * finalSpeed,
                            -gamepad1.left_stick_x * RobotConstants.strafeSpeed * finalSpeed,
                            -gamepad1.right_stick_x * RobotConstants.turnSpeed * finalSpeed,
                            maxSpeed, driveTrain.getPoseEstimate().getHeading(), driveTrain);

                    //Aligns robot to backboard if april tags have a detection
                    if (gamepad1.left_bumper && tagDetection) {
                        driveState.setValue(driveModeName + " Drive State: Re-localize");
                        driveStates = DriveStates.RELOCALIZE;
                    }
                    break;
                case RELOCALIZE:
                    //Setting the pose of the robot to pose detected by april tags
                    driveTrain.setPoseEstimate(new Pose2d(frontCamAprilTags.ftcPose.x, frontCamAprilTags.ftcPose.y, frontCamAprilTags.ftcPose.yaw));

                    //Setting target X of the robot
                    targetX = RobotConstants.backDropAlignX;

                    //Deciding which backboard to align to based on the robots pose
                    if (frontCamAprilTags.ftcPose.y>0) {
                        targetY = RobotConstants.backDropLeftY;
                    } else {
                        targetY = RobotConstants.backDropRightY;
                    }
                    driveState.setValue(driveModeName + " Drive State: Align");
                    driveStates = DriveStates.ALIGN;
                    break;
                case ALIGN:
                    //sets state back to default if drive1 exits the mode
                    if (!gamepad1.left_bumper) {
                        driveState.setValue(driveModeName + " Drive State: Drive");
                        driveStates = driveMode;
                    }

                    RobotMethods.goToPoint(targetX, targetY, 0, driveTrain);
                    if (abs(targetX-driveTrain.getPoseEstimate().getX())<.3 && abs(targetY-driveTrain.getPoseEstimate().getY())<.3) {
                        driveState.setValue(driveModeName + " Drive State: Hold");
                        driveStates = DriveStates.HOLD;
                    }
                    break;
                case HOLD:
                    //sets state back to default if drive1 exits the mode
                    if (!gamepad1.left_bumper) {
                        driveState.setValue(driveModeName + " Drive State: Drive");
                        driveStates = driveMode;
                    }

                    RobotMethods.goToLineY(targetX, gamepad1.left_stick_x * RobotConstants.strafeSpeed * finalSpeed, 0, driveTrain);
                    break;
                default:
                    driveState.setValue(driveModeName + " Drive State: Drive");
                    driveStates = driveMode;
            }


            //Driver 2 code
            //Code for dropping pixels out of outtake
            switch (drop) {
                case CLOSED:
                    if (gamepad2.left_bumper) {
                        robot.dropServo.setPosition(RobotConstants.dropOpen);
//                        robotPixelCount -= 1;
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
                        //Sets to clear so outtake will start checking for pixels again
//                        outtakeCount = OuttakeCount.CLEAR;
                    }
                    break;
                default:
                    drop = Drop.RESET;
            }

            //Code to change target position of slides
            if (abs(gamepad2.left_stick_y)>.1) {
                targetPos -= 8 * gamepad2.left_stick_y*slideTimer.milliseconds();
                targetPos = Range.clip(targetPos, RobotConstants.slideBottom, RobotConstants.slideTop);
            } else {
                switch (slidePos) {
                    case BOTTOM:
                        if (gamepad2.x) {
                            //                        roboMethods.setTargetPos(RobotConstants.slideBottom, RobotConstants.slideLow);
                            slidePos = Slide.LOW;
                            targetPos = RobotConstants.slideLow;
                        } else if (gamepad2.b) {
                            //                        roboMethods.setTargetPos(RobotConstants.slideBottom, RobotConstants.slideMiddle);
                            slidePos = Slide.MIDDLE;
                            targetPos = RobotConstants.slideMiddle;
                        } else if (gamepad2.y) {
                            //                        roboMethods.setTargetPos(RobotConstants.slideBottom, RobotConstants.slideTop);
                            slidePos = Slide.TOP;
                            targetPos = RobotConstants.slideTop;
                        }
                        break;
                    case LOW:
                        if (gamepad2.a) {
                            //                        roboMethods.setTargetPos(RobotConstants.slideLow, RobotConstants.slideBottom);
                            slidePos = Slide.BOTTOM;
                            targetPos = RobotConstants.slideBottom;
                        } else if (gamepad2.b) {
                            //                        roboMethods.setTargetPos(RobotConstants.slideLow, RobotConstants.slideMiddle);
                            slidePos = Slide.MIDDLE;
                            targetPos = RobotConstants.slideMiddle;
                        } else if (gamepad2.y) {
                            //                        roboMethods.setTargetPos(RobotConstants.slideLow, RobotConstants.slideTop);
                            slidePos = Slide.TOP;
                            targetPos = RobotConstants.slideTop;

                        }
                        break;
                    case MIDDLE:
                        if (gamepad2.a) {
                            //                        roboMethods.setTargetPos(RobotConstants.slideMiddle, RobotConstants.slideBottom);
                            slidePos = Slide.BOTTOM;
                            targetPos = RobotConstants.slideBottom;
                        } else if (gamepad2.x) {
                            //                        roboMethods.setTargetPos(RobotConstants.slideMiddle, RobotConstants.slideLow);
                            slidePos = Slide.LOW;
                            targetPos = RobotConstants.slideLow;
                        } else if (gamepad2.y) {
                            //                        roboMethods.setTargetPos(RobotConstants.slideMiddle, RobotConstants.slideTop);
                            slidePos = Slide.TOP;
                            targetPos = RobotConstants.slideTop;
                        }
                        break;
                    case TOP:
                        if (gamepad2.a) {
                            //                        roboMethods.setTargetPos(RobotConstants.slideTop, RobotConstants.slideBottom);
                            slidePos = Slide.BOTTOM;
                            targetPos = RobotConstants.slideBottom;
                        } else if (gamepad2.b) {
                            //                        roboMethods.setTargetPos(RobotConstants.slideTop, RobotConstants.slideMiddle);
                            slidePos = Slide.MIDDLE;
                            targetPos = RobotConstants.slideMiddle;
                        } else if (gamepad2.x) {
                            //                        roboMethods.setTargetPos(RobotConstants.slideTop, RobotConstants.slideLow);
                            slidePos = Slide.LOW;
                            targetPos = RobotConstants.slideLow;
                        }
                        break;
                }
            }
            slideTimer.reset();
            double slideVelo = robot.liftEncoder.getCorrectedVelocity();
            int slideCurPos = robot.liftEncoder.getCurrentPosition();

            double distRemain = targetPos - slideCurPos;

            slideI += distRemain * RobotConstants.slidePIDVals.i;

            double slidePower = (distRemain * RobotConstants.slidePIDVals.p) + slideI + (slideVelo * RobotConstants.slidePIDVals.d);

            robot.slideMotor.setPower(slidePower);

            slideData.setValue( "Encoder Val: " + slideCurPos + " Target Val: " + targetPos + " Slide Power: " + (double)Math.round(slidePower*100)/100);

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
                    if (climbTimer.milliseconds()>RobotConstants.climbReleaseDelay) {
//                        robot.climbMotor.setPower(0);
                        climb = Climb.WAIT;
                    }
                    break;
                case WAIT:
                    if (!gamepad1.dpad_down) {
                        climbTimer.reset();
                        climb = Climb.STOPPED;
                    }
                    break;
                case STOPPED:
                    if (gamepad1.dpad_down) {
                        targetClimbPos +=  2.5*climbTimer.milliseconds();
                        robot.climbMotor.setTargetPosition(targetClimbPos);
//                        climb = Climb.SPIN_IN;
                    } else if (gamepad1.dpad_up) {
                        targetClimbPos -=  2.5*climbTimer.milliseconds();
                        robot.climbMotor.setTargetPosition(targetClimbPos);
//                        climb = Climb.SPIN_OUT;
                    }
                    climbTimer.reset();
                    break;
//                case SPIN_IN:
//                    if (!gamepad1.dpad_down) {
//                        robot.climbMotor.setPower(0);
//                        climb = Climb.STOPPED;
//                    }
//                    break;
//                case SPIN_OUT:
//                    if (!gamepad1.dpad_up) {
//                        robot.climbMotor.setPower(0);
//                        climb = Climb.STOPPED;
//                    }
            }


            switch (intake) {
                case STOPPED:
                    if (gamepad2.dpad_down) {
                        robot.intakeMotor.setPower(RobotConstants.intakeSpeed);
                        intake = Spin.SPIN_IN;
                    } else if (gamepad2.dpad_up) {
                        robot.intakeMotor.setPower(-RobotConstants.intakeSpeed);
                        intake = Spin.SPIN_OUT;
                    }
                    break;
                case SPIN_IN:
                    if (!gamepad2.dpad_down) {
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

            switch (transfer) {
                case STOPPED:
                    if (gamepad2.dpad_right) {
                        robot.transferMotor.setPower(RobotConstants.transferSpeed);
                        transfer = Spin.SPIN_IN;
                    } else if (gamepad2.dpad_left) {
                        robot.transferMotor.setPower(-RobotConstants.transferSpeed);
                        transfer = Spin.SPIN_OUT;
                    }
                    break;
                case SPIN_IN:
                    if (!gamepad2.dpad_right) {
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

            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                robot.droneServo.setPosition(RobotConstants.droneRelease);
            }

            //Updating for roadrunner
            driveTrain.update();

            loopTime.setValue((double)Math.round(loopTimer.milliseconds()*100)/100 + "ms");

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