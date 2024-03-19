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

                    .followTrajectorySequence(drive -> (drive.trajectorySequenceBuilder(new Pose2d(16.7, -62, Math.toRadians(90)))).setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(310), 10.62))
                            .lineTo(new Vector2d(16.7, -45))
                            .splineToConstantHeading(new Vector2d(9, -35), Math.toRadians(180))
                            .lineTo(new Vector2d(8.5, -35))
                            .addTemporalMarker(1.5, () -> robot.leftPixelServo.setPosition(RobotConstants.leftIn))
                            .lineTo(new Vector2d(20, -35))
                            .splineToSplineHeading(new Pose2d(45, -29, Math.toRadians(0)), Math.toRadians(0))
                            .lineTo(new Vector2d(51.5, -29))
                            .addTemporalMarker(4.25, () -> {
                                robot.dropper(RobotConfig.Dropper.OPEN);
//                                backdropReading1.setValue(robot.safeRelocalizeBackdrop());
                            })
                            .waitSeconds(.3)
                            .back(.1)
                            .addTemporalMarker(5.2, () -> {
                                robot.setTargetSlidePos(RobotConfig.SlideHeight.BOTTOM);
                                robot.dropper(RobotConfig.Dropper.CLOSED);
                                robot.transferMotor.setPower(.3);
                            })
                            //cycle 1
                            .splineToConstantHeading(new Vector2d(24, -11.9), Math.toRadians(180))
                            .addTemporalMarker(7.5, () -> {
                                robot.intakeMotor.setPower(1);
                                robot.transferMotor.setPower(1);
                                robot.stackArm(RobotConfig.StackArm.OUT);
                            })
                            .lineTo(new Vector2d(-59.15, -11.9))
                            .waitSeconds(1)
                            .addTemporalMarker(8.25, () -> {
                                robot.grabFromStack(2);
                            })
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
