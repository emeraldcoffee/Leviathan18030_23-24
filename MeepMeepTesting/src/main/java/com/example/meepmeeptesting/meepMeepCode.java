package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class meepMeepCode {

    public static void main(String args[]) {


        MeepMeep meepMeep = new MeepMeep(800);
        RobotConfig robot = new RobotConfig();
//        RobotConstants RobotConstants = new RobotConstants();

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(40, 45, Math.toRadians(200), Math.toRadians(200), 10.64)
                .setColorScheme(new ColorSchemeBlueLight())

                    .followTrajectorySequence(drive -> (drive.trajectorySequenceBuilder(new Pose2d(-38, 62, Math.toRadians(270)))).setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(310), 10.62))
                            .splineToSplineHeading(new Pose2d(-39, 32.5, Math.toRadians(0)), Math.toRadians(270))
                            .addTemporalMarker(.1, () -> {
                                robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                            })
                            .lineTo(new Vector2d(-39, 37))
                            .addTemporalMarker(1.6, () -> {
                                robot.rightPixelServo.setPosition(RobotConstants.rightIn);
                            })
                            .back(.1)
                            .addTemporalMarker(2.2, () -> {
                                robot.intakeMotor.setPower(1);
                                robot.transferMotor.setPower(1);
                                robot.stackArm(RobotConfig.StackArm.OUT);
                            })
                            .splineToConstantHeading(new Vector2d(-57.4, 35.7), Math.toRadians(180))
                            .addTemporalMarker(4.0, () -> {
                                robot.grabFromStack(1);
                            })
                            .waitSeconds(.9)
                            .forward(.5)
                            .splineToConstantHeading(new Vector2d(-33, 57), Math.toRadians(0))
                            .lineTo(new Vector2d(31, 57))
                            .addTemporalMarker(7.2, () -> {
                                robot.setTargetSlidePos(RobotConfig.SlideHeight.PRELOAD_DROP);
                                robot.intakeMotor.setPower(0);
                                robot.transferMotor.setPower(0);
                            })
                            .splineToConstantHeading(new Vector2d(50, 35.5), Math.toRadians(0))
                            .lineTo(new Vector2d(52.5, 34.5), RobotConfig.getVelocityConstraint(25, Math.toRadians(200), 10.62), RobotConfig.getAccelerationConstraint(25))
                            .addTemporalMarker(9.2, () -> {
                                robot.dropper(RobotConfig.Dropper.OPEN);
                                robot.safeRelocalizeBackdrop();
                            })
                            .addTemporalMarker(9.7, () -> {
                                robot.setTargetSlidePos(RobotConfig.SlideHeight.LOW);
                                robot.dropper(RobotConfig.Dropper.PARTIAL);
                            })
                            .waitSeconds(1.3)
                            //Cycle 1
                            .back(.1)
                            .splineToConstantHeading(new Vector2d(33, 57), Math.toRadians(180))
                            .addTemporalMarker(11.1, () -> {
                                robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                            })
                            .lineTo(new Vector2d(-33, 57))
                            .splineToConstantHeading(new Vector2d(-52, 37.5), Math.toRadians(250))
                            .splineToConstantHeading(new Vector2d(-56.7, 34), Math.toRadians(180))
                            .addTemporalMarker(13.7, () -> {
                                robot.intakeMotor.setPower(1);
                                robot.transferMotor.setPower(1);
                                robot.dropper(RobotConfig.Dropper.CLOSED);
                                robot.stackArm(RobotConfig.StackArm.FAR_LEFT);
                            })
                            .waitSeconds(1.7)
                            .addTemporalMarker(15.0, () -> {
                                robot.stackArm(RobotConfig.StackArm.OUT);
                            })
                            .addTemporalMarker(15.3, () -> {
                                robot.grabFromStack(2);
                            })
                            //To backdrop
                            .forward(.5)
                            .splineToConstantHeading(new Vector2d(-33, 57), Math.toRadians(0))
                            .lineTo(new Vector2d(31, 57))
                            .addTemporalMarker(19.1, () -> {
                                robot.setTargetSlidePos(17);
                                robot.intakeMotor.setPower(0);
                                robot.transferMotor.setPower(0);
                            })
                            .splineToConstantHeading(new Vector2d(50, 37), Math.toRadians(0))
                            .lineTo(new Vector2d(52.5, 37), RobotConfig.getVelocityConstraint(25, Math.toRadians(200), 10.62), RobotConfig.getAccelerationConstraint(25))
                            .addTemporalMarker(21.7, () -> {
                                robot.dropper(RobotConfig.Dropper.PARTIAL);
                                robot.safeRelocalizeBackdrop();
                            })
                            .waitSeconds(1)
                            .addTemporalMarker(23.5, () -> {
                                robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                                robot.dropper(RobotConfig.Dropper.CLOSED);
                            })
                            .lineTo(new Vector2d(47, 37))
                            .lineTo(new Vector2d(47, PassData.parkPosition.blue))
                            .build());


//        .lineTo(new Vector2d(12, 60))
//                .splineToSplineHeading(new Pose2d(17, 30), Math.toRadians(270))
//                .lineTo(new Vector2d(17,36))
//                .addTemporalMarker(2.0, () -> {})//robot.rightServo.setPosition(RobotConstants.rightIn))
//                .lineTo(new Vector2d(18, 36))
//                .splineToConstantHeading(new Vector2d(45, 37.5), Math.toRadians(0))
//                .lineTo(new Vector2d(53, 37.5))
//                .addTemporalMarker(3.9, () -> {})//robot.dropServo.setPosition(RobotConstants.dropOpen))
//                .waitSeconds(.3)
//                .lineTo(new Vector2d(51, 37.5))
//                .addTemporalMarker(4, () -> {})
//                .splineToConstantHeading(new Vector2d(9, 12), Math.toRadians(180))
//                .lineTo(new Vector2d(-50, 12))


                // in front of trusses on blue alliance
//                .followTrajectorySequence(drive ->
//                    (drive.trajectorySequenceBuilder(new Pose2d(-35, 63, Math.toRadians(270)))
//
//                    .forward(28)
//                    // uses Vision to detect where the team prop is
//                    .addDisplacementMarker(() -> {
//
//                    })
//                    .waitSeconds(1)
//                    // places down pixel where team prop is
//                    .addDisplacementMarker(() -> {
//                        // turn depending on where the team prop is
//                        //RobotMethods.outtakePlace(hwMap);
//                    })
//                    .turn(Math.toRadians(90))
//
//                    .forward(84)
//                    // places down pixel on backdrop
//                    .addDisplacementMarker(() -> {
//                        //RobotMethods.slideExtend(hwMap, 5.0);
//                    })
//
//                    // cycling
//                    /*.waitSeconds(1)
//                    .forward(100)
//                    // picks up pixels
//                    .addDisplacementMarker(() -> {
//
//                    })
//                    .waitSeconds(1)
//                    .back(102)
//                    // place down pixel on backdrop
//                    .addDisplacementMarker(() -> {
//                        //RobotMethods.slideExtend(hwMap, 5.0);
//                    })*/
//                    .build()
//                    )
//                );

                // behind trusses on blue alliance
                /*.followTrajectorySequence(drive ->
                    (drive.trajectorySequenceBuilder(new Pose2d(12, 63, Math.toRadians(90)))
                            .back(19)

                            // uses Vision to detect where the team prop is
                            .addDisplacementMarker(() -> {})
                            .waitSeconds(1)

                            // places down pixel where team prop is
                            .addDisplacementMarker(() -> {
                                // turn depending on where the team prop is
                            })
                            .waitSeconds(1)
                            
                            .back(8)
                            .turn(Math.toRadians(90))
                            .back(40)
                            // places down pixel on backdrop
                            .addDisplacementMarker(() -> {})

                            .build()
                    )
                );

                /*.followTrajectorySequence(drive ->
                        // in front of trusses on red alliance
                        (drive.trajectorySequenceBuilder(new Pose2d(-35, -63, Math.toRadians(90)))
                                .forward(19)
                                .addDisplacementMarker(() -> {})
                                .waitSeconds(1)
                                .addDisplacementMarker(() -> {})
                                .waitSeconds(1)
                                .splineTo(new Vector2d(-12, -35), Math.toRadians(0))
                                .forward(60)
                                .build()
                        )
                );*/

                /*.followTrajectorySequence(drive ->
                        // behind trusses on red alliance
                        (drive.trajectorySequenceBuilder(new Pose2d(12, -63, Math.toRadians(90)))
                                .forward(20)
                                .addDisplacementMarker(() -> {}) // pick up pixel and use vision to detect positions
                                .waitSeconds(1)
                                .addDisplacementMarker(() -> {}) // places down corresponding pixels
                                .waitSeconds(1)
                                .splineTo(new Vector2d(30, -35), Math.toRadians(0))
                                .forward(18)
                                .build()
                        )
                );*/

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK).setDarkMode(true)
                // Background opacity from 0-1
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
