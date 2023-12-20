package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.HwMap;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "blue Close 3 Cycle Auto")
public class blueClose3Cycle extends LinearOpMode {

    String TSE_Position = "Right";

    boolean wait = true;

    enum FollowPath {
        SAVE,
        LEFT,
        RIGHT,
        CENTER,
        INIT_PATH,
        WAIT_DROP_LEFT,
        WAIT_DROP_CENTER,
        WAIT_DROP_RIGHT,
        WAIT_STACK1,
        C1_TO_BACKDROP,
        WAIT_DROP2,
        C2_TO_STACK,
        WAIT_STACK2,
        C2_TO_BACKDROP,
        WAIT_DROP3,
        C3_TO_STACK,
        WAIT_STACK3,
        C3_TO_BACKDROP,
        WAIT_DROP4,
        PARK
    }

    enum Intake {
        WAIT,
        INTAKE_DEPLOY,
        INTAKE_DEPLOY_ENDING_SEQUENCE,
        START_SPIN,
        PIXEL_A,
        PIXEL_B,
        TRANSFER_DELAY
    }
    Intake intake = Intake.WAIT;

    double intakePos = RobotConstants.stack5;

    enum Drop {
        WAIT,
        DROP,
        RESET
    }
    Drop drop = Drop.WAIT;

    FollowPath followPath  = FollowPath.SAVE;
    boolean pickup = false;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);
        HwMap robot = new HwMap();
        robot.init(hardwareMap);

        TrajectorySequence right = driveTrain.trajectorySequenceBuilder(new Pose2d(12, 63, Math.toRadians(270)))
                .lineTo(new Vector2d(12, 43))
                .splineToConstantHeading(new Vector2d(1, 34), Math.toRadians(180))
                .addSpatialMarker(new Vector2d(1.6, 34.3), () -> {})
                .lineTo(new Vector2d(5, 34))
                .splineToConstantHeading(new Vector2d(54.6,30), Math.toRadians(0))
                .build();

        TrajectorySequence rightToStack = driveTrain.trajectorySequenceBuilder(right.end())
                //Driving Back
                .lineTo(new Vector2d(54, 30))
                .splineToConstantHeading(new Vector2d(15, 12), Math.toRadians(180))
                .lineTo(new Vector2d(-30, 12))
                .splineToConstantHeading(new Vector2d(-55, 11), Math.toRadians(180))
                .build();

        TrajectorySequence center = driveTrain.trajectorySequenceBuilder(new Pose2d(12, 63, Math.toRadians(270)))
                .lineTo(new Vector2d(0, 0))
                .build();

        TrajectorySequence centerToStack = driveTrain.trajectorySequenceBuilder(new Pose2d(12, 63, Math.toRadians(270)))
                .lineTo(new Vector2d(0, 0))
                .build();

        TrajectorySequence left = driveTrain.trajectorySequenceBuilder(new Pose2d(12, 63, Math.toRadians(270)))
                .lineTo(new Vector2d(0, 0))
                .build();

        TrajectorySequence leftToStack = driveTrain.trajectorySequenceBuilder(new Pose2d(12, 63, Math.toRadians(270)))
                .lineTo(new Vector2d(0, 0))
                .build();



        TrajectorySequence leftStackReturn = driveTrain.trajectorySequenceBuilder(right.end())
                .lineTo(new Vector2d(-54, 11))
                .splineToConstantHeading(new Vector2d(-30, 12), Math.toRadians(0))
                .lineTo(new Vector2d(15, 12))
                .splineToConstantHeading(new Vector2d(54.6, 30), Math.toRadians(0))
                .build();

        TrajectorySequence leftStackPickup = driveTrain.trajectorySequenceBuilder(leftStackReturn.end())
                .lineTo(new Vector2d(54, 30))
                .splineToConstantHeading(new Vector2d(15, 12), Math.toRadians(180))
                .lineTo(new Vector2d(-30, 12))
                .splineToConstantHeading(new Vector2d(-55, 11), Math.toRadians(180))
                .build();

        TrajectorySequence centerStackPickup = driveTrain.trajectorySequenceBuilder(leftStackPickup.end())
                .lineTo(new Vector2d(54, 30))
                .splineToConstantHeading(new Vector2d(15, 12), Math.toRadians(180))
                .lineTo(new Vector2d(-30, 12))
                .splineToConstantHeading(new Vector2d(-55, 23), Math.toRadians(180))
                .build();

        TrajectorySequence centerReturn = driveTrain.trajectorySequenceBuilder(centerStackPickup.end())
                .lineTo(new Vector2d(-54, 23))
                .splineToConstantHeading(new Vector2d(-30, 12), Math.toRadians(0))
                .lineTo(new Vector2d(15, 12))
                .splineToConstantHeading(new Vector2d(54.6, 30), Math.toRadians(0))
                .build();

        TrajectorySequence park = driveTrain.trajectorySequenceBuilder(centerReturn.end())
                .waitSeconds(.2)
                .lineTo(new Vector2d(53, 28))
                .splineToConstantHeading(new Vector2d(55, 13), Math.toRadians(-20))
                .build();


        ElapsedTime followPathTimer = new ElapsedTime();
        ElapsedTime dropTimer = new ElapsedTime();
        ElapsedTime intakeTimer = new ElapsedTime();


        waitForStart();
        if (isStopRequested()) return;

//        cameraDelayTimer.reset();

        while (opModeIsActive() && !isStopRequested()) {

            switch (followPath) {
                case SAVE:
                    //Save camera data
                    switch (TSE_Position) {
                        case "Left":
                            driveTrain.followTrajectorySequenceAsync(left);
                            followPath = FollowPath.LEFT;
                            break;
                        case "Center":
                            driveTrain.followTrajectorySequenceAsync(center);
                            followPath = FollowPath.CENTER;
                        case "Right":
                            driveTrain.followTrajectorySequenceAsync(right);
                            followPath = FollowPath.RIGHT;

                    }
                    break;
                case LEFT:
                    if (!driveTrain.isBusy()) {
                        drop = Drop.DROP;
                        wait = true;
                        followPath = FollowPath.WAIT_DROP_LEFT;
                    }
                    break;
                case WAIT_DROP_LEFT:
                    if (!wait) {
                        driveTrain.followTrajectorySequenceAsync(leftToStack);
                        followPath = FollowPath.INIT_PATH;
                    }
                    break;
                case CENTER:
                    if (!driveTrain.isBusy()) {
                        drop = Drop.DROP;
                        wait = true;
                        followPath = FollowPath.WAIT_DROP_CENTER;
                    }
                    break;
                case WAIT_DROP_CENTER:
                    if (!wait) {
                        driveTrain.followTrajectorySequenceAsync(centerToStack);
                        followPath = FollowPath.INIT_PATH;
                    }
                    break;
                case RIGHT:
                    if (!driveTrain.isBusy()) {
                        drop = Drop.DROP;
                        wait = true;
                        followPath = FollowPath.WAIT_DROP_RIGHT;
                    }
                    break;
                case WAIT_DROP_RIGHT:
                    if (!wait) {
                        driveTrain.followTrajectorySequenceAsync(rightToStack);
                        followPath = FollowPath.INIT_PATH;
                    }
                    break;
                case INIT_PATH:
                    if (!driveTrain.isBusy()) {
                        intake = Intake.START_SPIN;
                        wait = true;
                        followPath = FollowPath.WAIT_STACK1;
                    }
                    break;
                case WAIT_STACK1:
                    if (!wait) {
                        driveTrain.followTrajectorySequenceAsync(leftStackReturn);
                        followPath = FollowPath.C1_TO_BACKDROP;
                    }
                    break;
                case C1_TO_BACKDROP:
                    if (!driveTrain.isBusy()) {
                        drop = Drop.DROP;
                        wait = true;
                        followPath = FollowPath.WAIT_DROP2;
                    }
                    break;
                case WAIT_DROP2:
                    if (!wait) {
                        driveTrain.followTrajectorySequenceAsync(park);
                        followPath = FollowPath.PARK;
                    }
                    break;
                case PARK:
                    break;

            }

            switch (intake) {
                case INTAKE_DEPLOY:
                    robot.transferMotor.setPower(.3);
                    robot.leftLiftServo.setPosition(intakePos);
                    robot.rightLiftServo.setPosition(intakePos);
                    intakeTimer.reset();
                    intake = Intake.INTAKE_DEPLOY_ENDING_SEQUENCE;
                    break;
                case INTAKE_DEPLOY_ENDING_SEQUENCE:
                    if (intakeTimer.seconds() > RobotConstants.intakeInitalizeDelay) {
                        robot.transferMotor.setPower(0);
                        intake = Intake.WAIT;
                    }
                    break;
                case START_SPIN:
                    robot.intakeMotor.setPower(1);
                    intake = Intake.PIXEL_A;
                    intakeTimer.reset();
                    break;
                case PIXEL_A:
                    //Later look into detecting the pixel with motor voltage
                    if (intakeTimer.seconds() > .5) {
                        if (intakePos == RobotConstants.stack5) {
                            intakePos = RobotConstants.stack4;
                        } else if (intakePos == RobotConstants.stack3) {
                            intakePos = RobotConstants.stack2;
                        }

                        robot.transferMotor.setPower(1);
                        robot.leftLiftServo.setPosition(intakePos+RobotConstants.stackLeftOffset);
                        robot.rightLiftServo.setPosition(intakePos);
                        intakeTimer.reset();
                        intake = Intake.PIXEL_B;
                    }
                    break;
                case PIXEL_B:
                    if (intakeTimer.seconds() > .5) {
                        intakeTimer.reset();
                        //Makes robot resume driving
                        wait = false;
                        intake = Intake.TRANSFER_DELAY;
                    }
                    break;
                case TRANSFER_DELAY:
                    if (intakeTimer.seconds() > RobotConstants.autoTransferTime) {
                        robot.transferMotor.setPower(0);
                        intake = Intake.WAIT;
                    }
                    break;
            }

            switch (drop) {
                case DROP:
                    robot.dropServo.setPosition(RobotConstants.dropPartial);
                    dropTimer.reset();
                    drop = Drop.RESET;
                    break;
                case RESET:
                    if (dropTimer.seconds() > RobotConstants.doubleDropTime) {
                        robot.dropServo.setPosition(RobotConstants.dropClosed);
                        wait = false;
                        drop = Drop.WAIT;
                    }
                    break;
            }

            driveTrain.update();
        }
    }

}
