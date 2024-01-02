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

    double intakePos = RobotConstants.stackMax;

    enum Drop {
        WAIT,
        DROP,
        RESET
    }
    Drop drop = Drop.WAIT;

    FollowPath followPath  = FollowPath.SAVE;
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

        TrajectorySequence right = driveTrain.trajectorySequenceBuilder(startingPose)
                .lineTo(new Vector2d(14, 43))
                .splineToConstantHeading(new Vector2d(1, 34), Math.toRadians(180))
                .addTemporalMarker(1.5, () -> robot.rightServo.setPosition(RobotConstants.rightIn))
                .addTemporalMarker(2.1, () -> targetSlidePos = RobotConstants.slideAuto)
                .lineTo(new Vector2d(5, 34))
                .splineToConstantHeading(new Vector2d(54.6,30), Math.toRadians(0))
                .build();

        TrajectorySequence rightToStack = driveTrain.trajectorySequenceBuilder(right.end())
                //Driving Back
                .lineTo(new Vector2d(54, 30))
                .splineToConstantHeading(new Vector2d(15, 9), Math.toRadians(180))
                .addTemporalMarker(1, () -> targetSlidePos = RobotConstants.slideBottom)
                .addTemporalMarker(1.2, () -> intake = Intake.INTAKE_DEPLOY)
                .lineTo(new Vector2d(-30, 9))
                .splineToConstantHeading(new Vector2d(-55, 11), Math.toRadians(180))
                .build();

        TrajectorySequence center = driveTrain.trajectorySequenceBuilder(startingPose)
                .lineTo(new Vector2d(12, 43))
                .splineToConstantHeading(new Vector2d(13, 25), Math.toRadians(270))
                .lineTo(new Vector2d(13, 32))
                .addTemporalMarker(1.8, () -> robot.rightServo.setPosition(RobotConstants.rightIn))
                .addTemporalMarker(2, () -> targetSlidePos = RobotConstants.slideAuto)
                .lineTo(new Vector2d(15, 32.5))
                .splineToConstantHeading(new Vector2d(54.6,36), Math.toRadians(0))
                .build();

        TrajectorySequence centerToStack = driveTrain.trajectorySequenceBuilder(center.end())
                .lineTo(new Vector2d(54, 36))
                .splineToConstantHeading(new Vector2d(15, 12), Math.toRadians(180))
                .addTemporalMarker(1, () -> targetSlidePos = RobotConstants.slideAuto)
                .addTemporalMarker(1.2, () -> intake = Intake.INTAKE_DEPLOY)
                .lineTo(new Vector2d(-30, 12))
                .splineToConstantHeading(new Vector2d(-55, 11), Math.toRadians(180))
                .build();

        TrajectorySequence left = driveTrain.trajectorySequenceBuilder(startingPose)
                .lineTo(new Vector2d(12, 48))
                .splineToConstantHeading(new Vector2d(22, 30), Math.toRadians(270))
                .lineTo(new Vector2d(22, 35))
                .addTemporalMarker(1.9, () -> robot.rightServo.setPosition(RobotConstants.rightIn))
                .addTemporalMarker(2.1, () -> targetSlidePos = RobotConstants.slideAuto)
                .lineTo(new Vector2d(24, 35.5))
                .splineToConstantHeading(new Vector2d(54.6,41), Math.toRadians(0))
                .build();

        TrajectorySequence leftToStack = driveTrain.trajectorySequenceBuilder(left.end())
                .lineTo(new Vector2d(54, 40.5))
                .splineToConstantHeading(new Vector2d(15, 12), Math.toRadians(180))
                .addTemporalMarker(1, () -> targetSlidePos = RobotConstants.slideBottom)
                .addTemporalMarker(1.2, () -> intake = Intake.INTAKE_DEPLOY)
                .lineTo(new Vector2d(-30, 12))
                .splineToConstantHeading(new Vector2d(-55, 11), Math.toRadians(180))
                .build();

        TrajectorySequence leftStackReturn = driveTrain.trajectorySequenceBuilder(rightToStack.end())
                .lineTo(new Vector2d(-54, 11))
                .splineToConstantHeading(new Vector2d(-30, 12), Math.toRadians(0))
                .lineTo(new Vector2d(15, 12))
                .addTemporalMarker(1.7, () -> targetSlidePos = RobotConstants.slideLow)
                .splineToConstantHeading(new Vector2d(54.6, 30), Math.toRadians(0))
                .build();

        TrajectorySequence leftStackPickup = driveTrain.trajectorySequenceBuilder(leftStackReturn.end())
                .lineTo(new Vector2d(54, 30))
                .splineToConstantHeading(new Vector2d(15, 12), Math.toRadians(180))
                .lineTo(new Vector2d(-30, 12))
                .addTemporalMarker(1, () -> targetSlidePos = RobotConstants.slideBottom)
                .splineToConstantHeading(new Vector2d(-55, 11), Math.toRadians(180))
                .build();

        TrajectorySequence centerStackPickup = driveTrain.trajectorySequenceBuilder(leftStackReturn.end())
                .lineTo(new Vector2d(54, 30))
                .splineToConstantHeading(new Vector2d(15, 12), Math.toRadians(180))
                .lineTo(new Vector2d(-30, 12))
                .addTemporalMarker(1, () -> targetSlidePos = RobotConstants.slideBottom)
                .splineToConstantHeading(new Vector2d(-55, 23), Math.toRadians(180))
                .build();

        TrajectorySequence centerReturn = driveTrain.trajectorySequenceBuilder(centerStackPickup.end())
                .lineTo(new Vector2d(-54, 23))
                .splineToConstantHeading(new Vector2d(-30, 12), Math.toRadians(0))
                .lineTo(new Vector2d(15, 12))
                .addTemporalMarker(1.9, () -> targetSlidePos = RobotConstants.slideLow)
                .splineToConstantHeading(new Vector2d(54.6, 30), Math.toRadians(0))
                .build();

        TrajectorySequence park = driveTrain.trajectorySequenceBuilder(centerReturn.end())
                .lineTo(new Vector2d(53, 28))
                .addTemporalMarker(.3, () -> targetSlidePos = RobotConstants.slideLow)
                .splineToConstantHeading(new Vector2d(55, 13), Math.toRadians(-20))
                .build();


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
                            followPath = FollowPath.LEFT;
                            break;
                        case "Center":
                            driveTrain.followTrajectorySequenceAsync(center);
                            followPath = FollowPath.CENTER;
                            break;
                        case "Right":
                            driveTrain.followTrajectorySequenceAsync(right);
                            followPath = FollowPath.RIGHT;
                            break;

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
                        intakePos = RobotConstants.stack5;
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
                    if (!driveTrain.isBusy()) {
                        loop = false;
                    }
                    break;

            }

            switch (intake) {
                case INTAKE_DEPLOY:
                    robot.transferMotor.setPower(.3);
                    robot.leftLiftServo.setPosition(intakePos+RobotConstants.stackLeftOffset);
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
                    robot.intakeMotor.setPower(-1);
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
