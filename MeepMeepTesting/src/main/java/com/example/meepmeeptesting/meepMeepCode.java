package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class meepMeepCode {
    public static void main(String args[]) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueLight())
                /*.followTrajectorySequence(drive ->
                        // in front of trusses on blue alliance
                        /*(drive.trajectorySequenceBuilder(new Pose2d(-35, 63, Math.toRadians(270)))
                                .strafeRight(19)
                                /*.addDisplacementMarker(() -> {}) // uses camera to look at AprilTags
                                .waitSeconds(1)
                                .turn(Math.toRadians(270))
                                .addDisplacementMarker(() -> {}) // places down corresponding pixels
                                .waitSeconds(1)
                                .splineTo(new Vector2d(-12, 35), Math.toRadians(0))
                                .forward(60)
                                .build()
                        )
                );*/

                .followTrajectorySequence(drive ->
                    // behind trusses on blue alliance
                    (drive.trajectorySequenceBuilder(new Pose2d(12, 63, Math.toRadians(0)))
                            .strafeRight(20)
                            .addDisplacementMarker(() -> {}) // uses camera to look at AprilTags
                            .waitSeconds(1)
                            .turn(Math.toRadians(270))
                            .addDisplacementMarker(() -> {}) // places down corresponding pixels
                            .waitSeconds(1)
                            .splineTo(new Vector2d(30, 35), Math.toRadians(0))
                            .forward(20)
                            .build()
                    )
                );

                /*.followTrajectorySequence(drive ->
                        // in front of trusses on red alliance
                        (drive.trajectorySequenceBuilder(new Pose2d(-35, -63, Math.toRadians(0)))
                                .strafeLeft(19)
                                .addDisplacementMarker(() -> {})
                                .waitSeconds(1)
                                .turn(Math.toRadians(90))
                                .addDisplacementMarker(() -> {})
                                .waitSeconds(1)
                                .splineTo(new Vector2d(-12, -35), Math.toRadians(0))
                                .forward(60)
                                .build()
                        )
                );*/

                /*.followTrajectorySequence(drive ->
                        // behind trusses on red alliance
                        (drive.trajectorySequenceBuilder(new Pose2d(12, -63, Math.toRadians(0)))
                                .strafeLeft(20)
                                .addDisplacementMarker(() -> {}) // uses camera to look at AprilTags
                                .waitSeconds(1)
                                .turn(Math.toRadians(90))
                                .addDisplacementMarker(() -> {}) // places down corresponding pixels
                                .waitSeconds(1)
                                .splineTo(new Vector2d(30, -35), Math.toRadians(0))
                                .forward(20)
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
