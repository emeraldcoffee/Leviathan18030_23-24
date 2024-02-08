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

@Autonomous(name = "red Close 3 Cycle Auto")
public class redClose3Cycle extends LinearOpMode {

    String TSE_Position = "Center";

    boolean wait = true;

    enum FollowPath {
        SAVE,
        FINISHED,
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

    final Pose2d startingPose = new Pose2d(12, -63, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);
        HwMap robot = new HwMap();
        robot.init(hardwareMap);

        robot.leftLiftServo.setPosition(intakePos+RobotConstants.stackLeftOffset);
        robot.rightLiftServo.setPosition(intakePos);

        //changed
        TrajectorySequence right = driveTrain.trajectorySequenceBuilder(startingPose)
                //to board
                .lineTo(new Vector2d(12, -61))
                .splineToLinearHeading(new Pose2d(23, -40), Math.toRadians(90))
                .lineTo(new Vector2d(23, -46))
                .addTemporalMarker(2.0, () -> robot.leftServo.setPosition(RobotConstants.leftIn))
                .addTemporalMarker(2.2, () -> targetSlidePos = RobotConstants.slideAuto)
                .lineTo(new Vector2d(26, -45))
                .splineToConstantHeading(new Vector2d(47, -45), Math.toRadians(0))
                .lineTo(new Vector2d(54.2, -45))
                .build();

        //changed
        TrajectorySequence rightToStack = driveTrain.trajectorySequenceBuilder(right.end())
                .lineTo(new Vector2d(54.1, -45))
                .splineToConstantHeading(new Vector2d(22, -11), Math.toRadians(180))
                .addTemporalMarker(1, () -> targetSlidePos = RobotConstants.slideBottom)
                .addTemporalMarker(1.2, () -> intake = redClose3Cycle.Intake.INTAKE_DEPLOY)
                .lineTo(new Vector2d(-56.9, -11))
                .splineToConstantHeading(new Vector2d(-34, -11), Math.toRadians(0))
                .build();
        //changed
        TrajectorySequence center = driveTrain.trajectorySequenceBuilder(startingPose)
                .lineTo(new Vector2d(12, -60))
                .splineToSplineHeading(new Pose2d(17, -30), Math.toRadians(90))
                .lineTo(new Vector2d(17, -36))
                .addTemporalMarker(2.0, () -> robot.leftServo.setPosition(RobotConstants.leftIn))
                .lineTo(new Vector2d(18, -36))
                .splineToConstantHeading(new Vector2d(45, -37.5), Math.toRadians(0))
                .lineTo(new Vector2d(54.2, -37.5))

                .build();
        //changed
        TrajectorySequence centerToStack = driveTrain.trajectorySequenceBuilder(center.end())
                .lineToConstantHeading(new Vector2d(54.1, -37.5))
                .splineToConstantHeading(new Vector2d(22, -11), Math.toRadians(180))
                .addTemporalMarker(1, () -> targetSlidePos = RobotConstants.slideBottom)
                .addTemporalMarker(1.2, () -> intake = redClose3Cycle.Intake.INTAKE_DEPLOY)
                .lineTo(new Vector2d(-56.9, -11))

                .build();

        //changed
        TrajectorySequence left = driveTrain.trajectorySequenceBuilder(startingPose)
                .lineTo(new Vector2d(12, -45))
                .splineToConstantHeading(new Vector2d(8.5, -36), Math.toRadians(180))
                .lineTo(new Vector2d(5, -36))
                .addTemporalMarker(1.6, () -> robot.leftServo.setPosition(RobotConstants.leftIn))
                .addTemporalMarker(2.0, () -> targetSlidePos = RobotConstants.slideAuto)
                .lineTo(new Vector2d(20, -36))
                .splineToSplineHeading(new Pose2d(45, -28.5, Math.toRadians(0)), Math.toRadians(0))
                .lineTo(new Vector2d(54.2, -28.5))
                .build();
        //changed
        TrajectorySequence leftToStack = driveTrain.trajectorySequenceBuilder(left.end())
                //Driving Back
                .lineToConstantHeading(new Vector2d(54.1, -28.5))
                .splineToConstantHeading(new Vector2d(22, -11), Math.toRadians(180))
                .addTemporalMarker(1, () -> targetSlidePos = RobotConstants.slideBottom)
                .addTemporalMarker(1.2, () -> intake = redClose3Cycle.Intake.INTAKE_DEPLOY)
                .lineTo(new Vector2d(-56.9, -11))
                .build();

        TrajectorySequence Return = driveTrain.trajectorySequenceBuilder(leftToStack.end())
                .lineTo(new Vector2d(-56.9, -11))
                .splineToConstantHeading(new Vector2d(-34, -9.7), Math.toRadians(0))
                .lineTo(new Vector2d(22, -9.7))
                .addTemporalMarker(2, () -> targetSlidePos = RobotConstants.slideAuto)
                .splineToConstantHeading(new Vector2d(54.3, -28.5), Math.toRadians(0))
                .build();



        TrajectorySequence park = driveTrain.trajectorySequenceBuilder(Return.end())
                .lineTo(new Vector2d(37, -12))
                .addTemporalMarker(.3, () -> targetSlidePos = RobotConstants.slideLow)
                .lineTo(new Vector2d(54, -12))

                /*.lineTo(new Vector2d(54.2, 30))
                .addTemporalMarker(.3, () -> targetSlidePos = RobotConstants.slideLow)
                .splineToConstantHeading(new Vector2d(54.2, 32), Math.toRadians(0))*/
                .build();


        ElapsedTime followPathTimer = new ElapsedTime();
        ElapsedTime dropTimer = new ElapsedTime();
        ElapsedTime intakeTimer = new ElapsedTime();

        Telemetry.Item slideData = telemetry.addData("Slide Data:", "Encoder Val:" + robot.liftEncoder.getCurrentPosition() + " Target Val:" + targetSlidePos);

        robot.rightServo.setPosition(RobotConstants.rightOut);

        waitForStart();
        if (isStopRequested()) return;
        driveTrain.update();
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
                        driveTrain.followTrajectorySequenceAsync(Return);
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
                    robot.rightLiftServo.setPosition(intakePos-0.06);
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
                        intake = Intake.PIXEL_B;
                    }
                    break;
                case PIXEL_B:
                    if (intakeTimer.seconds() > .7) {
                        intakeTimer.reset();
                        //Makes robot resume driving
                        wait = false;
                        //Spins intake
                        robot.transferMotor.setPower(1);
                        intake = Intake.TRANSFER_DELAY;
                    }
                    break;
                case TRANSFER_DELAY:
                    if (intakeTimer.seconds() > 4) {
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
/*
        TrajectorySequence park = driveTrain.trajectorySequenceBuilder(new Pose2d(-60, -11, Math.toRadians(180)))
                .lineTo(new Vector2d(37, -12))
                .waitSeconds(0.2)
                .lineTo(new Vector2d(54, -12))
                .build();


        TrajectorySequence cycleReturn = driveTrain.trajectorySequenceBuilder(new Pose2d(-57, -11, Math.toRadians(180)))
                //return
                .lineTo(new Vector2d(-56.9, -11))
                .splineToConstantHeading(new Vector2d(-34, -9.7), Math.toRadians(0))
                .lineTo(new Vector2d(22, -9.7))
                .addTemporalMarker(2, () -> targetSlidePos = RobotConstants.slideLow)
                .splineToConstantHeading(new Vector2d(54.3, -28.5), Math.toRadians(0))
                .addTemporalMarker(3.1, () -> robot.dropServo.setPosition(RobotConstants.dropPartial))
                .waitSeconds(0.3)

                //second
                .lineToConstantHeading(new Vector2d(54.2, -28.5))
                .splineToConstantHeading(new Vector2d(22, -9.7), Math.toRadians(180))

                .lineTo(new Vector2d(-34, -9.7))
                .splineToConstantHeading(new Vector2d(-57, -11), Math.toRadians(180))
                .addTemporalMarker(3, () -> {
                                    robot.transferMotor.setPower(.3);
                })
                .addTemporalMarker(4, () -> targetSlidePos = RobotConstants.slideBottom)

                .addTemporalMarker(5.4, () -> {
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide + RobotConstants.rightSpikeOffset);
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide);
                                    robot.intakeMotor.setPower(1);
                                    robot.transferMotor.setPower(1);
                                    robot.dropServo.setPosition(RobotConstants.dropClosed);
                })

                .addTemporalMarker(6.5, () -> {
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn + RobotConstants.rightSpikeOffset);
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                })
                .addTemporalMarker(7, () -> {
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide + RobotConstants.rightSpikeOffset);
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide);
                })
                .addTemporalMarker(7.5, () -> {
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn + RobotConstants.rightSpikeOffset);
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                })

                .waitSeconds(0.8)

                .lineTo(new Vector2d(-56.9, -11))
                .splineToConstantHeading(new Vector2d(-34, -9.7), Math.toRadians(0))
                .lineTo(new Vector2d(22, -9.7))
                .addTemporalMarker(2, () -> targetSlidePos = RobotConstants.slideLow)
                .splineToConstantHeading(new Vector2d(54.3, -28.5), Math.toRadians(0))
                .addTemporalMarker(3.1, () -> robot.dropServo.setPosition(RobotConstants.dropPartial))
                .waitSeconds(0.3)
                /*.addTemporalMarker(7.7, () -> {
                    driveTrain.followTrajectorySequenceAsync(cycleReturn);
                })*/
/*
                //third
                .lineToConstantHeading(new Vector2d(54.2, -28.5))
                .splineToConstantHeading(new Vector2d(22, -9.7), Math.toRadians(180))

                .lineTo(new Vector2d(-34, -9.7))
                .splineToConstantHeading(new Vector2d(-57, -11), Math.toRadians(180))
                .addTemporalMarker(3, () -> {
                                    robot.transferMotor.setPower(.3);
                })
                .addTemporalMarker(4, () -> targetSlidePos = RobotConstants.slideBottom)

                .addTemporalMarker(5.4, () -> {
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide + RobotConstants.rightSpikeOffset);
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide);
                                    robot.intakeMotor.setPower(1);
                                    robot.transferMotor.setPower(1);
                                    robot.dropServo.setPosition(RobotConstants.dropClosed);
                })

                .addTemporalMarker(6.5, () -> {
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn + RobotConstants.rightSpikeOffset);
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                })
                .addTemporalMarker(7, () -> {
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide + RobotConstants.rightSpikeOffset);
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide);
                })
                .addTemporalMarker(7.5, () -> {
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn + RobotConstants.rightSpikeOffset);
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                })
                .waitSeconds(0.8)

                .lineTo(new Vector2d(-56.9, -11))
                .splineToConstantHeading(new Vector2d(-34, -9.7), Math.toRadians(0))
                .lineTo(new Vector2d(22, -9.7))
                .addTemporalMarker(2, () -> targetSlidePos = RobotConstants.slideLow)
                .splineToConstantHeading(new Vector2d(54.3, -28.5), Math.toRadians(0))
                .addTemporalMarker(3.1, () -> robot.dropServo.setPosition(RobotConstants.dropPartial))
                .waitSeconds(0.3)

                //fourth
                /*.lineToConstantHeading(new Vector2d(54.2, -28.5))
                .splineToConstantHeading(new Vector2d(22, -8.3), Math.toRadians(180))

                .lineTo(new Vector2d(-34, -8.3))
                .splineToConstantHeading(new Vector2d(-60, -11), Math.toRadians(180))
                .addTemporalMarker(3, () -> {
                    robot.transferMotor.setPower(.3);
                })
                .addTemporalMarker(4, () -> targetSlidePos = RobotConstants.slideBottom)

                .addTemporalMarker(5.4, () -> {
                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide + RobotConstants.rightSpikeOffset);
                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide);
                    robot.intakeMotor.setPower(1);
                    robot.transferMotor.setPower(1);
                    robot.dropServo.setPosition(RobotConstants.dropClosed);
                })

                .addTemporalMarker(6.5, () -> {
                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn + RobotConstants.rightSpikeOffset);
                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                })
                .addTemporalMarker(7, () -> {
                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide + RobotConstants.rightSpikeOffset);
                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide);
                })
                .addTemporalMarker(7.5, () -> {
                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn + RobotConstants.rightSpikeOffset);
                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                })
                .waitSeconds(0.2)*/



                //park
/*
                .lineTo(new Vector2d(37, -12))
                .waitSeconds(0.2)
                .lineTo(new Vector2d(54, -12))


                .build();

        TrajectorySequence goToStackFromPath = driveTrain.trajectorySequenceBuilder(new Pose2d(22, -9.7, Math.toRadians(180)))
                .lineTo(new Vector2d(-34, -9.7))
                .splineToConstantHeading(new Vector2d(-57, -11), Math.toRadians(180))
                .addTemporalMarker(3, () -> {
                                    robot.transferMotor.setPower(.3);
                })
                .addTemporalMarker(4, () -> targetSlidePos = RobotConstants.slideBottom)

                .addTemporalMarker(5.4, () -> {
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide + RobotConstants.rightSpikeOffset);
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide);
                                    robot.intakeMotor.setPower(1);
                                    robot.transferMotor.setPower(1);
                                    robot.dropServo.setPosition(RobotConstants.dropClosed);
                })

                .addTemporalMarker(6.5, () -> {
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn + RobotConstants.rightSpikeOffset);
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                })
                .addTemporalMarker(7, () -> {
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide + RobotConstants.rightSpikeOffset);
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkGuide);
                })
                .addTemporalMarker(7.5, () -> {
                                    robot.rightSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn + RobotConstants.rightSpikeOffset);
                                    robot.leftSpikeMarkServo.setPosition(RobotConstants.spikeMarkIn);
                })

                .addTemporalMarker(7.7, () -> {
                            driveTrain.followTrajectorySequenceAsync(cycleReturn);
                })
                .build();

        //once you grab something from the stack and want to do it again, follow in the format of continueCycle, goToStackFromPath, and then return cycle
        TrajectorySequence continueCycle = driveTrain.trajectorySequenceBuilder(new Pose2d(54.3, -28.5, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(54.2, -28.5))
                .splineToConstantHeading(new Vector2d(19, -6.6), Math.toRadians(180))

                //call goToStack from path

                .build();


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
                .waitSeconds(.2)
                .lineToConstantHeading(new Vector2d(54.2, -28.5))
                .splineToConstantHeading(new Vector2d(22, -9.7), Math.toRadians(180))
                .addTemporalMarker(5.62, () -> {
                        driveTrain.followTrajectorySequenceAsync(goToStackFromPath);
                })
                .build();

        TrajectorySequence center = driveTrain.trajectorySequenceBuilder(new Pose2d(12, -63, Math.toRadians(90)))
                .lineTo(new Vector2d(12, -60))
                .splineToSplineHeading(new Pose2d(17, -30), Math.toRadians(90))
                .lineTo(new Vector2d(17, -36))
                .addTemporalMarker(2.0, () -> robot.leftServo.setPosition(RobotConstants.leftIn))
                .lineTo(new Vector2d(18, -36))
                .splineToConstantHeading(new Vector2d(45, -37.5), Math.toRadians(0))
                .lineTo(new Vector2d(54.2, -37.5))
                .lineTo(new Vector2d(54.3, -37.5))
                .addTemporalMarker(3.9, () -> robot.dropServo.setPosition(RobotConstants.dropOpen))
                .waitSeconds(.2)
                .lineToConstantHeading(new Vector2d(54.2, -37.5))
                .splineToConstantHeading(new Vector2d(22, -9.7), Math.toRadians(180))
                .addTemporalMarker(5.99, () -> {
                    driveTrain.followTrajectorySequenceAsync(goToStackFromPath);
                })
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
                .lineToConstantHeading(new Vector2d(54.2, -45))
                .splineToConstantHeading(new Vector2d(22, -9.7), Math.toRadians(180))
                .addTemporalMarker(5.98, () -> {
                    driveTrain.followTrajectorySequenceAsync(goToStackFromPath);
                })
                .build();


        ElapsedTime cameraDelayTimer = new ElapsedTime();

        telemetry.setAutoClear(false);
        Telemetry.Item detectedPos = telemetry.addData("Position", "No detection");
        Telemetry.Item wasPos = telemetry.addData("Was pos", "");
        Telemetry.Item slideData = telemetry.addData("Slide Data:", "Encoder Val:" + robot.liftEncoder.getCurrentPosition() + " Target Val:" + targetSlidePos);

        robot.rightServo.setPosition(RobotConstants.rightOut);
        robot.dropServo.setPosition(RobotConstants.dropClosed);

        waitForStart();
        if (isStopRequested()) return;

        robot.climbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveTrain.update();
        driveTrain.setPoseEstimate(new Pose2d(12, 63.5, Math.toRadians(270)));

        cameraDelayTimer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            driveTrain.update();

            switch (camera) {
                case WAIT:
                    if (cameraDelayTimer.seconds() > 1.5) {
                        camera = redClose3Cycle.Camera.SAVE;
                    }
                    break;
                case SAVE:
                    pos = pipeline.getPos();
                    switch (pos) {
                        case "left":
                            driveTrain.followTrajectorySequenceAsync(left);
                            detectedPos.setValue("Left");
                            break;
                        case "center":
                            driveTrain.followTrajectorySequenceAsync(center);
                            detectedPos.setValue("Center");
                            break;
                        case "right":
                            driveTrain.followTrajectorySequenceAsync(right);
                            detectedPos.setValue("Right");
                            break;
                        default:
                            driveTrain.followTrajectorySequenceAsync(center);
                            detectedPos.setValue("Default center (No detection)");
                            break;

                    }
                    wasPos.setValue(RobotMethods.updateRobotPosition(driveTrain.getPoseEstimate()));

                    camera = redClose3Cycle.Camera.FINISHED;
                    break;
                case FINISHED:
                    break;
            }

            double slideVelo = robot.liftEncoder.getCorrectedVelocity();
            int slideCurPos = robot.liftEncoder.getCurrentPosition();

            double distRemain = targetSlidePos - slideCurPos;

            slideI += distRemain * RobotConstants.slidePIDVals.i;

            double slidePower = (distRemain * RobotConstants.slidePIDVals.p) + slideI + (slideVelo * RobotConstants.slidePIDVals.d);

            robot.slideMotor.setPower(slidePower);

            slideData.setValue("Encoder Val: " + slideCurPos + " Target Val: " + targetSlidePos + " Slide Power: " + (double) Math.round(slidePower * 100) / 100);

            //Limits max speed servos move
           /* if (drawbridgeTargetPos<drawbridgeCurrentPos) {
              drawbridgeCurrentPos+= Range.clip((drawbridgeTargetPos-drawbridgeCurrentPos), -.015, -.0);
               robot.rightDrawbridgeServo.setPosition(drawbridgeCurrentPos+RobotConstants.drawbridgeRightOffset);
               robot.leftDrawbridgeServo.setPosition(drawbridgeCurrentPos);
           } else if (drawbridgeTargetPos>drawbridgeCurrentPos) {
                drawbridgeCurrentPos+=Range.clip((drawbridgeTargetPos-drawbridgeCurrentPos), 0, .015);
               robot.rightDrawbridgeServo.setPosition(drawbridgeCurrentPos+RobotConstants.drawbridgeRightOffset);
               robot.leftDrawbridgeServo.setPosition(drawbridgeCurrentPos);

*/