package org.firstinspires.ftc.teamcode.robot;


import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp()
public class testTele extends LinearOpMode {

    //Establish variables
    double maxSpeed = 1;

    enum Drop {
        OPEN,
        CLOSED,
        RESET
    }

    enum Slide {
        RETRACTED,

        EXTENDED,

        RESET
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //Updating Status
        Telemetry.Item status = telemetry.addData("Status", "Initializing");
        telemetry.update();

        //Init code
        hardwareMap robot = new hardwareMap();
        robot.init(hardwareMap);
        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);

        ElapsedTime dropTimer = new ElapsedTime();
        ElapsedTime slideTimer = new ElapsedTime();
        Drop drop = Drop.CLOSED;

        Slide slide = Slide.RETRACTED;

        //Getting last pose
        driveTrain.setPoseEstimate(PassData.currentPose);

        //Adding roadrunner pose to telemetry
        Telemetry.Item robotPose = telemetry.addData("Robot pose:", RobotMethods.updateRobotPosition(driveTrain.getPoseEstimate()));
        telemetry.update();

        //Set starting positions
        robot.dropServo.setPosition(RobotConstants.dropClosed);

        //Updating Status
        status.setValue("Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        status.setValue("Running");

        while(opModeIsActive() && !isStopRequested()) {
            //Getting robots estimated position
            Pose2d myPose = driveTrain.getPoseEstimate();
            //Setting telemetry to display robots position
            robotPose.setValue(RobotMethods.updateRobotPosition(myPose));

            //Driver 1 code

            //Front triggers being used to speedup or slowdown robots driving
            double finalSpeed = RobotConstants.speedMultiplier * (1+(gamepad1.right_trigger-gamepad1.left_trigger)/1.2);

            //Getting joystick values for driving
            double drive = gamepad1.left_stick_y * RobotConstants.driveSpeed * finalSpeed;
            double strafe = gamepad1.left_stick_x * RobotConstants.strafeSpeed * finalSpeed;
            double turn = gamepad1.right_stick_x * RobotConstants.turnSpeed * finalSpeed;

            //Calculating and applying the powers for mecanum wheels
            //For field-centric driving replace below line with: robotMethods.setMecanumDriveFieldCentric(drive, strafe, turn, maxSpeed, myPose.getHeading(), driveTrain);
            RobotMethods.setMecanumDrive(drive, strafe, turn, maxSpeed, driveTrain);


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

            switch (slide) {
                case RETRACTED: // this needs to slowly let go of the slides to let the counterweight take over
                    if (gamepad2.right_bumper) { // this if statement needs to be outside in a loop, if bumper, then slide enum = EXTENDED
                        slideTimer.reset();
                        slide = Slide.EXTENDED;
                    }
                    break;
                case EXTENDED:
                    robot.transferMotor.setPower(0.2);

                    if (slideTimer.seconds() > 2) { // slideTimer preferably needs to start timing when EXTENDED starts, like while loop (while (slideTimer.seconds() < 2))
                        robot.transferMotor.setPower(0);
                        slide = Slide.RETRACTED;
                    }
                    break;
                default:
                    slide = Slide.RETRACTED;

            }


            if (gamepad1.dpad_down) {
                robot.intakeMotor.setPower(RobotConstants.intakeSpeed);

            }
            else if (gamepad1.dpad_right) {
                robot.transferMotor.setPower(RobotConstants.transferSpeed);
            }

            //Updating telemetry
            telemetry.update();

            //Updating for roadrunner
            driveTrain.update();

        }
        //Passing robots estimated position when tele is stopped
        PassData.currentPose = driveTrain.getPoseEstimate();
    }


}
