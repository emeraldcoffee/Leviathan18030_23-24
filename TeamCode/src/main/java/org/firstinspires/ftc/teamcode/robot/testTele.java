package org.firstinspires.ftc.teamcode.robot;


import static org.firstinspires.ftc.teamcode.robot.RobotConstants.slidePIDVals;
import static org.firstinspires.ftc.teamcode.robot.testTele.Climb.RELEASE;

import android.widget.Switch;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.robocol.TelemetryMessage;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;


@TeleOp()
public class testTele extends LinearOpMode {

    //Establish variables
    double maxSpeed = 1;
    RobotMethods roboMethods;

    int targetPos = RobotConstants.slideBottom;

    enum DriveType {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }

    enum Drop {
        OPEN,
        CLOSED,
        RESET
    }

    enum Slide {
        BOTTOM,
        LOW,
        MIDDLE,
        TOP
    }

    enum Climb {
        RELEASE,
        WAIT,
        CLIMB
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //Updating Status
        Telemetry.Item status = telemetry.addData("Status", "Initializing");
        telemetry.update();

        //Init code
        roboMethods = new RobotMethods();
        HwMap robot = new HwMap();
        robot.init(hardwareMap);
        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);

        DriveType driveType = DriveType.ROBOT_CENTRIC;

        ElapsedTime dropTimer = new ElapsedTime();
        Drop drop = Drop.CLOSED;

        ElapsedTime climbTimer = new ElapsedTime();
        Climb climb = RELEASE;

        ElapsedTime slideTimer = new ElapsedTime();
        Slide slidePos = Slide.BOTTOM;
        double slideI = 0.0;

        //Getting last pose
        driveTrain.setPoseEstimate(PassData.currentPose);

        telemetry.setAutoClear(false);
        Telemetry.Item driveMode = telemetry.addData("Drive Mode", "Robot Centric");

        //Adding roadrunner pose to telemetry
        Telemetry.Item robotPose = telemetry.addData("Robot pose:", RobotMethods.updateRobotPosition(driveTrain.getPoseEstimate()));

        //Adding odom pod encoders to telemetry
        Telemetry.Item odom = telemetry.addData("Encoder Positions:", StandardTrackingWheelLocalizer.getEncoderVals());
        Telemetry.Item slideData = telemetry.addData("Slide Data:", robot.liftEncoder.getCurrentPosition());
        telemetry.update();



        //Set starting positions
        robot.dropServo.setPosition(RobotConstants.dropClosed);

        //Updating Status
        status.setValue("Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        status.setValue("Running");
        roboMethods.setTargetPos(robot.liftEncoder.getCurrentPosition(), RobotConstants.slideBottom);

        while (opModeIsActive() && !isStopRequested()) {
            //Getting robots estimated position
            Pose2d myPose = driveTrain.getPoseEstimate();
            //Setting telemetry to display robots position
            robotPose.setValue(RobotMethods.updateRobotPosition(myPose));

            //Driver 1 code

            //Front triggers being used to speedup or slowdown robots driving
            double finalSpeed = RobotConstants.speedMultiplier * (1 + (gamepad1.right_trigger - gamepad1.left_trigger) / 1.2);

            //Getting joystick values for driving
            double drive = gamepad1.left_stick_y * RobotConstants.driveSpeed * finalSpeed;
            double strafe = -(gamepad1.left_stick_x * RobotConstants.strafeSpeed * finalSpeed);
            double turn = gamepad1.right_stick_x * RobotConstants.turnSpeed * finalSpeed;


            //Adding ability to switch between field and drive centric
            switch (driveType) {
                case ROBOT_CENTRIC:
                    //Calculating and applying the powers for mecanum wheels
                    //For field-centric driving replace below line with: robotMethods.setMecanumDriveFieldCentric(drive, strafe, turn, maxSpeed, myPose.getHeading(), driveTrain);
                    RobotMethods.setMecanumDrive(drive, strafe, turn, maxSpeed, driveTrain);

                    if (gamepad1.left_bumper && gamepad1.dpad_right) {
                        driveMode.setValue("Field Centric");
                        driveType = DriveType.FIELD_CENTRIC;
                    }
                    break;
                case FIELD_CENTRIC:
                    RobotMethods.setMecanumDriveFieldCentric(drive, strafe, turn, maxSpeed, myPose.getHeading(), driveTrain);

                    if (gamepad1.left_bumper && gamepad1.dpad_left) {
                        driveMode.setValue("Robot Centric");
                        driveType = DriveType.ROBOT_CENTRIC;
                    }
                    break;
            }

            //Driver 2 code

            //Code for dropping pixels out of outtake
            switch (drop) {
                case CLOSED:
                    if (gamepad2.left_bumper) {
                        robot.dropServo.setPosition(RobotConstants.dropOpen);
                        dropTimer.reset();
                        drop = Drop.OPEN;
                    }
                    break;
                case OPEN:
                    if (dropTimer.seconds() > RobotConstants.dropTime) {
                        robot.dropServo.setPosition(RobotConstants.dropClosed);
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


            switch (slidePos) {
                case BOTTOM:
                    if (gamepad2.x) {
                        roboMethods.setTargetPos(RobotConstants.slideBottom, RobotConstants.slideLow);
                        targetPos = RobotConstants.slideLow;
                        slidePos = Slide.LOW;
                    }
                    else if (gamepad2.b) {
                        roboMethods.setTargetPos(RobotConstants.slideBottom, RobotConstants.slideMiddle);
                        targetPos = RobotConstants.slideMiddle;
                        slidePos = Slide.MIDDLE;
                    }
//                    else if (gamepad2.y) {
//                        roboMethods.setTargetPos(RobotConstants.slideBottom, RobotConstants.slideTop);
//                        targetPos = RobotConstants.slideTop;
//                        slidePos = Slide.TOP;
//                    }
                    break;
                case LOW:
                    if (gamepad2.a) {
                        roboMethods.setTargetPos(RobotConstants.slideLow, RobotConstants.slideBottom);
                        targetPos = RobotConstants.slideBottom;
                        slidePos = Slide.BOTTOM;
                    }
                    else if (gamepad2.b) {
                        roboMethods.setTargetPos(RobotConstants.slideLow, RobotConstants.slideMiddle);
                        targetPos = RobotConstants.slideMiddle;
                        slidePos = Slide.MIDDLE;
                    }
//                    else if (gamepad2.y) {
//                        roboMethods.setTargetPos(RobotConstants.slideLow, RobotConstants.slideTop);
//                        targetPos = RobotConstants.slideTop;
//                        slidePos = Slide.TOP;
//                    }
                    break;
                case MIDDLE:
                    if (gamepad2.a) {
                        roboMethods.setTargetPos(RobotConstants.slideMiddle, RobotConstants.slideBottom);
                        targetPos = RobotConstants.slideBottom;
                        slidePos = Slide.BOTTOM;
                    }
                    else if (gamepad2.x) {
                        roboMethods.setTargetPos(RobotConstants.slideMiddle, RobotConstants.slideLow);
                        targetPos = RobotConstants.slideLow;
                        slidePos = Slide.LOW;
                    }
//                    else if (gamepad2.y) {
//                        roboMethods.setTargetPos(RobotConstants.slideMiddle, RobotConstants.slideTop);
//                        targetPos = RobotConstants.slideTop;
//                        slidePos = Slide.TOP;
//                    }
                    break;
                case TOP:
                    if (gamepad2.a) {
                        roboMethods.setTargetPos(RobotConstants.slideTop, RobotConstants.slideBottom);
                        targetPos = RobotConstants.slideBottom;
                        slidePos = Slide.BOTTOM;
                    }
                    else if (gamepad2.b) {
                        roboMethods.setTargetPos(RobotConstants.slideTop, RobotConstants.slideMiddle);
                        targetPos = RobotConstants.slideMiddle;
                        slidePos = Slide.MIDDLE;
                    }
                    else if (gamepad2.x) {
                        roboMethods.setTargetPos(RobotConstants.slideTop, RobotConstants.slideLow);
                        targetPos = RobotConstants.slideLow;
                        slidePos = Slide.LOW;
                    }
                    break;
            }

            double slideVelo = robot.liftEncoder.getCorrectedVelocity();
            int slideCurPos = robot.liftEncoder.getCurrentPosition();

            targetPos = Range.clip(targetPos, RobotConstants.slideMinHeight, RobotConstants.slideMaxHeight);

            double distRemain = targetPos - slideCurPos;

            slideI += distRemain * slidePIDVals.i;


            double slidePower = (distRemain * slidePIDVals.p) + slideI + (slideVelo * slidePIDVals.d);

            robot.liftMotor.setPower(slidePower);

            /*if (slideTimer.seconds() > 2) { // slideTimer preferably needs to start timing when EXTENDED starts, like while loop (while (slideTimer.seconds() < 2))
                robot.liftMotor.setPower(-(distRemain * slidePIDVals.p) + slideI + (slideVelo * slidePIDVals.d));
            }*/

            // intake
            if (gamepad2.dpad_down) {
                robot.intakeMotor.setPower(RobotConstants.intakeSpeed);
            } else if (gamepad2.dpad_up) {
                robot.intakeMotor.setPower(-RobotConstants.intakeSpeed);
            } else {
                robot.intakeMotor.setPower(0);
            }

            if (gamepad2.dpad_right) {
                robot.transferMotor.setPower(RobotConstants.transferSpeed);
            } else if(gamepad2.dpad_left) {
                robot.transferMotor.setPower(-RobotConstants.transferSpeed);
            }  else {
                robot.transferMotor.setPower(0);
            }


//            switch (climb) {
//                case RELEASE:
//                    if (gamepad1.a) {
//                        robot.climbMotor.setPower(1);
//                        climbTimer.reset();
//                        climb = Climb.WAIT;
//                    }
//                    break;
//                case WAIT:
//                    if (climbTimer.seconds()>.3) {
//                        climb = Climb.CLIMB;
//                    }
//                    break;
//                case CLIMB:
//                    if (gamepad1.b) {
//                        robot.climbMotor.setPower(1);
//                    }
//            }

            if(gamepad1.x) {
                robot.climbMotor.setPower(-1);
            } else if(gamepad1.b) {
                robot.climbMotor.setPower(1);
            } else {
                robot.climbMotor.setPower(0);
            }

//            if ((gamepad2.left_trigger != 0.0) && (gamepad2.right_trigger != 0.0)) {
//                robot.climbMotor.setPower(RobotConstants.climbSpeed);
//            }
//            else {
//                robot.climbMotor.setPower(0);
//            }

            //Updating Telemetry Items
            robotPose.setValue(RobotMethods.updateRobotPosition(myPose));

            odom.setValue(StandardTrackingWheelLocalizer.getEncoderVals());

            slideData.setValue(RobotMethods.slideData(slideCurPos, targetPos, slidePower, distRemain * slidePIDVals.p, slideI, slideVelo * slidePIDVals.d));


            //Updating telemetry
            telemetry.update();

            //Updating for roadrunner
            driveTrain.update();
        }
        robot.liftMotor.setPower(0.0);

//            switch (slide) {
//                case RETRACTED: // this needs to slowly let go of the slides to let the counterweight take over
//                    if (gamepad2.right_bumper) { // this if statement needs to be outside in a loop, if bumper, then slide enum = EXTENDED
//                        RobotMethods.slideExtend(robot, 50);
//                        slide = Slide.EXTENDED;
//                    }
//                    break;
//                case EXTENDED:
//                    robot.climbMotor.setPower(0.2);
//
//                    if (slideTimer.seconds() > 2) { // slideTimer preferably needs to start timing when EXTENDED starts, like while loop (while (slideTimer.seconds() < 2))
//                        robot.climbMotor.setPower(0);
//                        slide = Slide.RETRACTED;
//                    }
//                    break;
//                default:
//                    slide = Slide.RETRACTED;
//
//            }
    }
}
