package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.HwMap;
import org.firstinspires.ftc.teamcode.robot.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


public class redClose3Cycle {


    String TSE_Position = "Center";

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
    redClose3Cycle.Intake intake = redClose3Cycle.Intake.WAIT;

    double intakePos = RobotConstants.stackMax;

    enum Drop {
        WAIT,
        DROP,
        RESET
    }
    redClose3Cycle.Drop drop = redClose3Cycle.Drop.WAIT;

    redClose3Cycle.FollowPath followPath  = redClose3Cycle.FollowPath.SAVE;
    boolean pickup = false;

    boolean loop = true;

    int targetSlidePos = RobotConstants.slideBottom;

    double slideI = 0;

    final Pose2d startingPose = new Pose2d(12, 63, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);
        HwMap robot = new HwMap();
        robot.init(hardwareMap);

        robot.leftLiftServo.setPosition(intakePos+RobotConstants.stackLeftOffset);
        robot.rightLiftServo.setPosition(intakePos);

        Intake intake = Intake.WAIT;

        TrajectorySequence left = driveTrain.trajectorySequenceBuilder(new Pose2d(12, -63, Math.toRadians(90)))
                .lineTo(new Vector2d(12, -45))
                .splineToConstantHeading(new Vector2d(8.5, -36), Math.toRadians(180))
                .lineTo(new Vector2d(5, -36))
                .addTemporalMarker(1.6, () -> robot.leftServo.setPosition(RobotConstants.leftIn))
                .lineTo(new Vector2d(20, -36))
                .splineToSplineHeading(new Pose2d(45, -28.5, Math.toRadians(0)), Math.toRadians(0))
                .lineTo(new Vector2d(54.2, -28.5))
                .lineTo(new Vector2d(54.3, -28.5))
                .addTemporalMarker(4, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
                .waitSeconds(.3)
                .build());

            TrajectorySequence leftToStack1 = driveTrain.trajectoryBuilder(left.end())
                    .lineToConstantHeading(new Vector2d(54.2,-28.5))
                    .splineToConstantHeading(new Vector2d(19, -6.6), Math.toRadians(180))
                    .lineTo(new Vector2d(-34, -6.6))
                    .splineToConstantHeading(new Vector2d(-60,-11), Math.toRadians(180))
                    .addTemporalMarker(1, () -> {})//targetSlidePos = RobotConstants.slideBottom)
                    //reaches stack here
                    .addTemporalMarker(1.2, () -> {})//intake = Intake.INTAKE_DEPLOY)
                    .lineToConstantHeading(new Vector2d(-59.9, -11))
                    .splineToConstantHeading(new Vector2d(-34,-6.6), Math.toRadians(0))
                    .lineTo(new Vector2d(19, -6.6))
                    .splineToConstantHeading(new Vector2d(54.3, -28.5), Math.toRadians(0))
                    //reaches backdrop
                    .addTemporalMarker(3.9, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))

            TrajectorySequence leftToStackAfter = driveTrain.trajectoryBuilder(leftToStack1.end())
                    //go back to reset vector direction
                    .lineToConstantHeading(new Vector2d(54.2, -28.5))
                    .splineToConstantHeading(new Vector2d(19, -6.6), Math.toRadians(180))
                    .lineTo(new Vector2d(-34, -6.6))
                    .splineToConstantHeading(new Vector2d(-60,-11), Math.toRadians(180))
                    .addTemporalMarker(1, () -> targetSlidePos = RobotConstants.slideBottom)
                    //reaches stack here
                    .addTemporalMarker(1.2, () -> intake = Intake.INTAKE_DEPLOY)
                    .lineToConstantHeading(new Vector2d(-59.9, -11))
                    .splineToConstantHeading(new Vector2d(-34,-6.6), Math.toRadians(0))
                    .lineTo(new Vector2d(19, -6.6))
                    .splineToConstantHeading(new Vector2d(54.3, -28.5), Math.toRadians(0))
                    //reaches backdrop
                    .addTemporalMarker(3.9, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))

            TrajectorySequence center = driveTrain.trajectorySequenceBuilder(new Pose2d(12, -63, Math.toRadians(90)))
                .lineTo(new Vector2d(12, -60))
                .splineToSplineHeading(new Pose2d(17, -30), Math.toRadians(90))
                .lineTo(new Vector2d(17,-36))
                .addTemporalMarker(2.0, () -> robot.leftServo.setPosition(RobotConstants.leftIn))
                .lineTo(new Vector2d(18, -36))
                .splineToConstantHeading(new Vector2d(45, -37.5), Math.toRadians(0))
                .lineTo(new Vector2d(54.2, -37.5))
                .lineTo(new Vector2d(54.3, -37.5))
                .addTemporalMarker(3.9, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
                .waitSeconds(.3)
                .build();

        TrajectorySequence centerToStack1 = driveTrain.trajectorySequenceBuilder(center.end())
                .lineToConstantHeading(new Vector2d(54.1,-37.5))
                .splineToConstantHeading(new Vector2d(19, -6.6), Math.toRadians(180))
                .lineTo(new Vector2d(-34, -6.6))
                .splineToConstantHeading(new Vector2d(-60,-11), Math.toRadians(180))
                .addTemporalMarker(1, () -> targetSlidePos = RobotConstants.slideBottom)
                //reaches stack here
                .addTemporalMarker(1.2, () -> intake = Intake.INTAKE_DEPLOY)
                .lineToConstantHeading(new Vector2d(-59.9, -11))
                .splineToConstantHeading(new Vector2d(-34,-6.6), Math.toRadians(0))
                .lineTo(new Vector2d(19, -6.6))
                .splineToConstantHeading(new Vector2d(54.3, -30), Math.toRadians(0))
                //reaches backdrop
                .addTemporalMarker(3.9, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
                //reset the spline to a new direction
                .build();

        TrajectorySequence centerToStackAfter = driveTrain.trajectorySequenceBuilder(centerToStack1.end())
                .lineToConstantHeading(new Vector2d(54.2, -30))
                .splineToConstantHeading(new Vector2d(19, -6.6), Math.toRadians(180))
                .lineTo(new Vector2d(-34, -6.6))
                .splineToConstantHeading(new Vector2d(-60,-11), Math.toRadians(180))
                .addTemporalMarker(1, () -> targetSlidePos = RobotConstants.slideBottom)
                //reaches stack here
                .addTemporalMarker(1.2, () -> intake = Intake.INTAKE_DEPLOY)
                .lineToConstantHeading(new Vector2d(-59.9, -11))
                .splineToConstantHeading(new Vector2d(-34,-6.6), Math.toRadians(0))
                .lineTo(new Vector2d(19, -6.6))
                .splineToConstantHeading(new Vector2d(54.3, -30), Math.toRadians(0))
                //reaches backdrop
                .addTemporalMarker(3.9, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
                .build();

        TrajectorySequence right = driveTrain.trajectorySequenceBuilder(new Pose2d(12, -63, Math.toRadians(90)))
                .lineTo(new Vector2d(12, -61))
                .splineToLinearHeading(new Pose2d(23, -40), Math.toRadians(90))
                .lineTo(new Vector2d(23, -46))
                .addTemporalMarker(2.0, () -> robot.leftServo.setPosition(RobotConstants.leftIn))
                .lineTo(new Vector2d(26, -45))
                .splineToConstantHeading(new Vector2d(47, -45), Math.toRadians(0))
                .lineTo(new Vector2d(54.2, -45))
                .lineTo(new Vector2d(54.3, -45))
                .addTemporalMarker(3.8, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
                .waitSeconds(.2)
                .build();

        TrajectorySequence rightToStack1 = driveTrain.trajectorySequenceBuilder(right.end())
                //Driving Back
                .lineTo(new Vector2d(54.2, -45))
                .splineToSplineHeading(new Pose2d(17, -6.6), Math.toRadians(180))
                .lineTo(new Vector2d(-34,-6.6))
                .splineToConstantHeading(new Vector2d(-60,-11), Math.toRadians(180))
                //reaches stack
                .addTemporalMarker(2.0, () -> {})//robot.leftServo.setPosition(RobotConstants.leftIn))
                .lineToConstantHeading(new Vector2d(-59.9, -11))
                .splineToConstantHeading(new Vector2d(-34, -6.6), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(17, -6.6))
                .splineToConstantHeading(new Vector2d(54.3, -30), Math.toRadians(0))
                //reached backdrop
                .build();
        //use after the first round of backToStack
        TrajectorySequence rightToStackAfter = driveTrain.trajectorySequenceBuilder(rightToStack1.end())
                .lineToConstantHeading(new Vector2d(54.2, -30))
                //.lineTo(new Vector2d(54.2, -37.5))
                .splineToSplineHeading(new Pose2d(17, -6.6), Math.toRadians(180))
                .lineTo(new Vector2d(-34,-6.6))
                .splineToConstantHeading(new Vector2d(-60,-11), Math.toRadians(180))
                //reaches stack
                .addTemporalMarker(2.0, () -> {})//robot.leftServo.setPosition(RobotConstants.leftIn))
                .lineToConstantHeading(new Vector2d(-59.9, -11))
                .splineToConstantHeading(new Vector2d(-34, -6.6), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(17, -6.6))
                .splineToConstantHeading(new Vector2d(54.3, -30), Math.toRadians(0))

        ElapsedTime followPathTimer = new ElapsedTime();
        ElapsedTime dropTimer = new ElapsedTime();
        ElapsedTime intakeTimer = new ElapsedTime();

        Telemetry.Item slideData = telemetry.addData("Slide Data:", "Encoder Val:" + robot.liftEncoder.getCurrentPosition() + " Target Val:" + targetSlidePos);

        robot.rightServo.setPosition(RobotConstants.rightOut);

        waitForStart();
        if (isStopRequested()) return;

        driveTrain.setPoseEstimate(startingPose);

//        cameraDelayTimer.reset();

        while (opModeIsActive() && !isStopRequested() && loop) {

            switch (followPath) {
                case SAVE:
                    //Save camera data
                    switch (TSE_Position) {
                        case "Left":
                            driveTrain.followTrajectorySequenceAsync(left);
                            followPath = redClose3Cycle.FollowPath.LEFT;
                            break;
                        case "Center":
                            driveTrain.followTrajectorySequenceAsync(center);
                            followPath = redClose3Cycle.FollowPath.CENTER;
                            break;
                        case "Right":
                            driveTrain.followTrajectorySequenceAsync(right);
                            followPath = redClose3Cycle.FollowPath.RIGHT;
                            break;

                    }
                    break;
                case LEFT:
                    if (!driveTrain.isBusy()) {
                        drop = redClose3Cycle.Drop.DROP;
                        wait = true;
                        followPath = redClose3Cycle.FollowPath.WAIT_DROP_LEFT;
                    }
                    break;
                case WAIT_DROP_LEFT:
                    if (!wait) {
                        driveTrain.followTrajectorySequenceAsync(leftToStack);
                        followPath = redClose3Cycle.FollowPath.INIT_PATH;
                    }
                    break;
                case CENTER:
                    if (!driveTrain.isBusy()) {
                        drop = redClose3Cycle.Drop.DROP;
                        wait = true;
                        followPath = redClose3Cycle.FollowPath.WAIT_DROP_CENTER;
                    }
                    break;
                case WAIT_DROP_CENTER:
                    if (!wait) {
                        driveTrain.followTrajectorySequenceAsync(centerToStack);
                        followPath = redClose3Cycle.FollowPath.INIT_PATH;
                    }
                    break;
                case RIGHT:
                    if (!driveTrain.isBusy()) {
                        drop = redClose3Cycle.Drop.DROP;
                        wait = true;
                        followPath = redClose3Cycle.FollowPath.WAIT_DROP_RIGHT;
                    }
                    break;
                case WAIT_DROP_RIGHT:
                    if (!wait) {
                        driveTrain.followTrajectorySequenceAsync(rightToStack);
                        followPath = redClose3Cycle.FollowPath.INIT_PATH;
                    }
                    break;
                case INIT_PATH:
                    if (!driveTrain.isBusy()) {
                        intakePos = RobotConstants.stack5;
                        intake = redClose3Cycle.Intake.START_SPIN;
                        wait = true;
                        followPath = redClose3Cycle.FollowPath.WAIT_STACK1;
                    }
                    break;
                case WAIT_STACK1:
                    if (!wait) {
                        driveTrain.followTrajectorySequenceAsync(leftStackReturn);
                        followPath = redClose3Cycle.FollowPath.C1_TO_BACKDROP;
                    }
                    break;
                case C1_TO_BACKDROP:
                    if (!driveTrain.isBusy()) {
                        drop = redClose3Cycle.Drop.DROP;
                        wait = true;
                        followPath = redClose3Cycle.FollowPath.WAIT_DROP2;
                    }
                    break;
                case WAIT_DROP2:
                    if (!wait) {
                        driveTrain.followTrajectorySequenceAsync(park);
                        followPath = redClose3Cycle.FollowPath.PARK;
                    }
                    break;
                case PARK:
                    if (!driveTrain.isBusy()) {
                        loop = false;
                    }
                    break;

            }

            switch (intake) {
                case INTAKE_DEPLOY:
                    robot.transferMotor.setPower(.3);
                    robot.leftLiftServo.setPosition(intakePos+RobotConstants.stackLeftOffset);
                    robot.rightLiftServo.setPosition(intakePos-0.06);
                    intakeTimer.reset();
                    intake = redClose3Cycle.Intake.INTAKE_DEPLOY_ENDING_SEQUENCE;
                    break;
                case INTAKE_DEPLOY_ENDING_SEQUENCE:
                    if (intakeTimer.seconds() > RobotConstants.intakeInitalizeDelay) {
                        robot.transferMotor.setPower(0);
                        intake = redClose3Cycle.Intake.WAIT;
                    }
                    break;
                case START_SPIN:
                    robot.intakeMotor.setPower(-1);
                    intake = redClose3Cycle.Intake.PIXEL_A;
                    intakeTimer.reset();
                    break;
                case PIXEL_A:
                    //Later look into detecting the pixel with motor voltage
                    if (intakeTimer.seconds() > .7) {
                        if (intakePos == RobotConstants.stack5) {
                            intakePos = RobotConstants.stack4;
                        } else if (intakePos == RobotConstants.stack4) {
                            intakePos = RobotConstants.stack3;
                        } else if (intakePos == RobotConstants.stack3) {
                            intakePos = RobotConstants.stack2;
                        } else if (intakePos == RobotConstants.stack2) {
                            intakePos = RobotConstants.stack1;
                        }

                        robot.transferMotor.setPower(1);
                        robot.leftLiftServo.setPosition(intakePos+RobotConstants.stackLeftOffset);
                        robot.rightLiftServo.setPosition(intakePos-0.04);
                        intakeTimer.reset();
                        intake = redClose3Cycle.Intake.PIXEL_B;
                    }
                    break;
                case PIXEL_B:
                    if (intakeTimer.seconds() > .7) {
                        intakeTimer.reset();
                        //Makes robot resume driving
                        wait = false;
                        intake = redClose3Cycle.Intake.TRANSFER_DELAY;
                    }
                    break;
                case TRANSFER_DELAY:
                    if (intakeTimer.seconds() > RobotConstants.autoTransferTime) {
                        robot.transferMotor.setPower(0);
                        intake = redClose3Cycle.Intake.WAIT;
                    }
                    break;
            }

            switch (drop) {
                case DROP:
                    robot.dropServo.setPosition(RobotConstants.dropPartial);
                    dropTimer.reset();
                    drop = redClose3Cycle.Drop.RESET;
                    break;
                case RESET:
                    if (dropTimer.seconds() > RobotConstants.doubleDropTime) {
                        robot.dropServo.setPosition(RobotConstants.dropClosed);
                        wait = false;
                        drop = redClose3Cycle.Drop.WAIT;
                    }
                    break;
            }

            //Slide pid
            double slideVelo = robot.liftEncoder.getCorrectedVelocity();
            int slideCurPos = robot.liftEncoder.getCurrentPosition();

            double distRemain = targetSlidePos - slideCurPos;

            slideI += distRemain * RobotConstants.slidePIDVals.i;

            double slidePower = (distRemain * RobotConstants.slidePIDVals.p) + slideI + (slideVelo * RobotConstants.slidePIDVals.d);

            robot.slideMotor.setPower(slidePower);

            slideData.setValue( "Encoder Val: " + slideCurPos + " Target Val: " + targetSlidePos + " Slide Power: " + (double)Math.round(slidePower*100)/100);


            driveTrain.update();
        }
        robot.rightServo.setPosition(RobotConstants.rightIn);
        robot.slideMotor.setPower(0);
        //Saving robots pose
        PoseStorage.currentPose = driveTrain.getPoseEstimate();
    }

}


