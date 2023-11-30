package org.firstinspires.ftc.teamcode.robot;


import static java.lang.Math.abs;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;


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

    enum Drop {
        OPEN,
        CLOSED,
        RESET
    }

    //Using AtomicReference bc variables are accessed by multiple threads
    //True when color black is not sensed in outtake
    AtomicReference<Boolean > outtakePixel = new AtomicReference<>(false);
    short outtakePixelCount = 0;
    //True when distance sensor reads less than certain value
    AtomicReference<Boolean> intakePixel = new AtomicReference<>(false);
    short intakePixelCount = 0;

    enum Slide {
        BOTTOM,
        LOW,
        MIDDLE,
        TOP
    }
    int targetPos = RobotConstants.slideBottom;

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

        AprilTagProcessor frontAprilTagProcessor = null;
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

        //Used to store data from april tags
        AprilTagDetection frontCamAprilTags = null;

        //Used to tell if camera has detected april tags
        boolean tagDetection;

        DriveStates driveStates = driveMode;

        ElapsedTime dropTimer = new ElapsedTime();
        Drop drop = Drop.CLOSED;

        ElapsedTime slideTimer = new ElapsedTime();
        Slide slidePos = Slide.BOTTOM;
        final PIDCoefficients slidePIDVals = new PIDCoefficients(2.0 / 8192, .01 / 8192, .001 / 8192);
        double slideI = 0.0;

        Runnable updateColorSensor = () -> outtakePixel.set(robot.outtakeColorSensor.red() > RobotConstants.outtakeValue);

        Runnable updateDistanceSenor = () -> intakePixel.set(robot.intakeDistanceSensor.getDistance(DistanceUnit.CM)< RobotConstants.intakeValue);

        //Getting last pose
        driveTrain.setPoseEstimate(PassData.currentPose);

        //Adding roadrunner pose to telemetry
        Telemetry.Item robotPose = telemetry.addData("Robot pose:", RobotMethods.updateRobotPosition(driveTrain.getPoseEstimate()));

        //Adding odom pod encoders to telemetry
        Telemetry.Item odom = telemetry.addData("Encoder Positions:", StandardTrackingWheelLocalizer.getEncoderVals());

        Telemetry.Item aprilTagPosEstimate = telemetry.addData("April-tag Estimated Pos:", "");

        //Set starting positions
        //robot.dropServo.setPosition(RobotConstants.dropClosed);



        //Updating Status
        status.setValue("Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        status.setValue("Running");

        while (opModeIsActive() && !isStopRequested()) {

            //Threads read i2c sensors and update variables async bc reading the sensors can be slow
            updateColorSensor.run();
            updateDistanceSenor.run();

            //Getting robots estimated position
            Pose2d myPose = driveTrain.getPoseEstimate();
            //Setting telemetry to display robots position
            robotPose.setValue(RobotMethods.updateRobotPosition(myPose));

            //Getting aprilTag detections
            if (frontAprilTagProcessor.getDetections().size() > 0) {
                //Gets all the april tag data for the 1st detection
                frontCamAprilTags = frontAprilTagProcessor.getDetections().get(0);
                aprilTagPosEstimate.setValue(RobotMethods.updateRobotPosAprilTag(frontCamAprilTags));
                tagDetection = true;
            } else {
                aprilTagPosEstimate.setValue("No tags detected");
                tagDetection = false;
            }


            //Driver 1 code
            //Front triggers being used to speedup or slowdown robots driving
            double finalSpeed = RobotConstants.speedMultiplier * (1 + (gamepad1.right_trigger - gamepad1.left_trigger) / 1.2);

            //Calculating and applying the powers for mecanum wheels
            //For field-centric driving replace below line with: robotMethods.setMecanumDriveFieldCentric(drive, strafe, turn, maxSpeed, myPose.getHeading(), driveTrain);
            switch (driveStates) {
                case GAMEPAD:
                    //Setting drive speeds for the robot
                    RobotMethods.setMecanumDrive(gamepad1.left_stick_y * RobotConstants.driveSpeed * finalSpeed,
                            gamepad1.left_stick_x * RobotConstants.strafeSpeed * finalSpeed,
                            gamepad1.right_stick_x * RobotConstants.turnSpeed * finalSpeed,
                            maxSpeed, driveTrain);

                    //Aligns robot to backboard if april tags have a detection
                    if (gamepad1.left_bumper && tagDetection) {
                        driveStates = DriveStates.RELOCALIZE;
                    }
                    break;
                case GAMEPAD_FIELDCENTRIC:
                    //Setting drive speeds for the robot
                    RobotMethods.setMecanumDriveFieldCentric(gamepad1.left_stick_y * RobotConstants.driveSpeed * finalSpeed,
                            gamepad1.left_stick_x * RobotConstants.strafeSpeed * finalSpeed,
                            gamepad1.right_stick_x * RobotConstants.turnSpeed * finalSpeed,
                            maxSpeed, driveTrain.getPoseEstimate().getHeading(), driveTrain);

                    //Aligns robot to backboard if april tags have a detection
                    if (gamepad1.left_bumper && tagDetection) {
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
                    driveStates = DriveStates.ALIGN;
                    break;
                case ALIGN:
                    //sets state back to default if drive1 exits the mode
                    if (!gamepad1.left_bumper) {
                        driveStates = driveMode;
                    }

                    RobotMethods.goToPoint(targetX, targetY, 0, driveTrain);
                    if (abs(targetX-driveTrain.getPoseEstimate().getX())<.3 && abs(targetY-driveTrain.getPoseEstimate().getY())<.3) {
                        driveStates = DriveStates.HOLD;
                    }
                    break;
                case HOLD:
                    //sets state back to default if drive1 exits the mode
                    if (!gamepad1.left_bumper) {
                        driveStates = driveMode;
                    }

                    RobotMethods.goToLineY(targetX, gamepad1.left_stick_x * RobotConstants.strafeSpeed * finalSpeed, 0, driveTrain);
                    break;
                default:
                    driveStates = driveMode;
            }


            //Driver 2 code
            //Code for dropping pixels out of outtake
            switch (drop) {
                case CLOSED:
                    if (gamepad2.left_bumper) {
                        //robot.dropServo.setPosition(RobotConstants.dropOpen);
                        dropTimer.reset();
                        drop = Drop.OPEN;
                    }
                    break;
                case OPEN:
                    if (dropTimer.seconds() > RobotConstants.dropTime) {
                        //robot.dropServo.setPosition(RobotConstants.dropClosed);
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

            if (abs(gamepad2.left_stick_y)>.1) {
                targetPos -= 10*gamepad2.left_stick_y;
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

            double slideVelo = robot.liftEncoder.getCorrectedVelocity();
            int slideCurPos = robot.liftEncoder.getCurrentPosition();

            double distRemain = targetPos - slideCurPos;

            slideI += distRemain * slidePIDVals.i;

            robot.slideMotor.setPower((distRemain * slidePIDVals.p) + slideI + (slideVelo * slidePIDVals.d));


            if (gamepad2.dpad_down) {
                robot.intakeMotor.setPower(RobotConstants.intakeSpeed);
            } else {
                robot.intakeMotor.setPower(0);
            }

            if (gamepad2.dpad_right) {
                robot.transferMotor.setPower(RobotConstants.transferSpeed);
            } else {
                robot.transferMotor.setPower(0);
            }

            //Updating telemetry
            telemetry.update();

            //Updating for roadrunner
            driveTrain.update();
        }
        robot.slideMotor.setPower(0.0);

    }


}
