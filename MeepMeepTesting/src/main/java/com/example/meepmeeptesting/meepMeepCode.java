package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class meepMeepCode {
    public static void main(String args[]) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(310), Math.toRadians(250), 10.62)
                .setColorScheme(new ColorSchemeBlueLight())

                        .followTrajectorySequence(drive -> (drive.trajectorySequenceBuilder(new Pose2d(54.6, 30, Math.toRadians(0))))
                                .lineTo(new Vector2d(54, 40.5))
                                .splineToConstantHeading(new Vector2d(15, 12), Math.toRadians(180))
                                .addTemporalMarker(1, () -> {})//targetSlidePos = RobotConstants.slideBottom)
                                .lineTo(new Vector2d(-30, 12))
                                .splineToConstantHeading(new Vector2d(-55, 11), Math.toRadians(180))


                                .build());

        /*
        .lineTo(new Vector2d(-35, 47))
                                .splineToConstantHeading(new Vector2d(-20,35), Math.toRadians(0))
                                .lineTo(new Vector2d(-28, 35))
                                .splineToConstantHeading(new Vector2d(-36, 48), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(-20, 60), Math.toRadians(0))
                                .lineTo(new Vector2d(13, 60))
                                .splineToConstantHeading(new Vector2d(54.6, 42), Math.toRadians(0))
                                .addSpatialMarker(new Vector2d(54.6, 42), () -> {})
                                .waitSeconds(.2)
                                //Driving back
                                .lineTo(new Vector2d(54, 42))
                                .splineToConstantHeading(new Vector2d(13, 60), Math.toRadians(180))
                                .lineTo(new Vector2d(-20, 60))
                                .splineToConstantHeading(new Vector2d(-55, 35.5), Math.toRadians(180))
                                .waitSeconds(1)
                                .lineTo(new Vector2d(-54, 35.5))
                                .splineToConstantHeading(new Vector2d(-20, 60), Math.toRadians(0))
                                .lineTo(new Vector2d(13, 60))
                                .splineToConstantHeading(new Vector2d(54.6, 42), Math.toRadians(0))
                                .waitSeconds(.2)

                                //Cycle 2
                                .lineTo(new Vector2d(54, 42))
                                .splineToConstantHeading(new Vector2d(13, 60), Math.toRadians(180))
                                .lineTo(new Vector2d(-20, 60))
                                .splineToConstantHeading(new Vector2d(-55, 35.5), Math.toRadians(180))
                                .waitSeconds(1)
                                .lineTo(new Vector2d(-54, 35.5))
                                .splineToConstantHeading(new Vector2d(-20, 60), Math.toRadians(0))
                                .lineTo(new Vector2d(13, 60))
                                .splineToConstantHeading(new Vector2d(54.6, 42), Math.toRadians(0))
                                .waitSeconds(.2)
                                //Cycle 3
                                .lineTo(new Vector2d(54, 42))
                                .splineToConstantHeading(new Vector2d(13, 60), Math.toRadians(180))
                                .lineTo(new Vector2d(-20, 60))
                                .splineToConstantHeading(new Vector2d(-55, 23.5), Math.toRadians(180))
                                .waitSeconds(1)
                                .lineTo(new Vector2d(-54, 23.5))
                                .splineToConstantHeading(new Vector2d(-20, 60), Math.toRadians(0))
                                .lineTo(new Vector2d(13, 60))
                                .splineToConstantHeading(new Vector2d(54.6, 42), Math.toRadians(0))
                                .waitSeconds(.2)
                                .lineTo(new Vector2d(53, 44))
                                .splineToConstantHeading(new Vector2d(55, 58), Math.toRadians(20))
         */

        /*close 3 cycle auto
        .lineTo(new Vector2d(12, 43))
                                .splineToConstantHeading(new Vector2d(1, 34), Math.toRadians(180))
                                .lineTo(new Vector2d(5, 34))
                                .splineToConstantHeading(new Vector2d(54.6,30), Math.toRadians(0))
                                .waitSeconds(.2)
                                //Driving Back
                                .lineTo(new Vector2d(54, 30))
                                .splineToConstantHeading(new Vector2d(15, 12), Math.toRadians(180))
                                .lineTo(new Vector2d(-30, 12))
                                .splineToConstantHeading(new Vector2d(-55, 11), Math.toRadians(180))
                                .waitSeconds(1)
                                .lineTo(new Vector2d(-54, 11))
                                .splineToConstantHeading(new Vector2d(-30, 12), Math.toRadians(0))
                                .lineTo(new Vector2d(15, 12))
                                .splineToConstantHeading(new Vector2d(54.6, 30), Math.toRadians(0))
                                .waitSeconds(.2)
                                //Cycle 2
                                .lineTo(new Vector2d(54, 30))
                                .splineToConstantHeading(new Vector2d(15, 12), Math.toRadians(180))
                                .lineTo(new Vector2d(-30, 12))
                                .splineToConstantHeading(new Vector2d(-55, 11), Math.toRadians(180))
                                .waitSeconds(1)
                                .lineTo(new Vector2d(-54, 11))
                                .splineToConstantHeading(new Vector2d(-30, 12), Math.toRadians(0))
                                .lineTo(new Vector2d(15, 12))
                                .splineToConstantHeading(new Vector2d(54.6, 30), Math.toRadians(0))
                                .waitSeconds(.2)
                                //Cycle 3
                                .lineTo(new Vector2d(54, 30))
                                .splineToConstantHeading(new Vector2d(15, 12), Math.toRadians(180))
                                .lineTo(new Vector2d(-30, 12))
                                .splineToConstantHeading(new Vector2d(-55, 23), Math.toRadians(180))
                                .waitSeconds(1)
                                .lineTo(new Vector2d(-54, 23))
                                .splineToConstantHeading(new Vector2d(-30, 12), Math.toRadians(0))
                                .lineTo(new Vector2d(15, 12))
                                .splineToConstantHeading(new Vector2d(54.6, 30), Math.toRadians(0))
                                .waitSeconds(.2)
                                .lineTo(new Vector2d(53, 28))
                                .splineToConstantHeading(new Vector2d(55, 13), Math.toRadians(-20))
         */

        /*
        .lineTo(new Vector2d(13, 62))
                                .splineToConstantHeading(new Vector2d(54.6, 34), Math.toRadians(0))
                                .waitSeconds(.2)
                                .lineTo(new Vector2d(54, 34))
                                .splineToConstantHeading(new Vector2d(22, 32), Math.toRadians(180))
                                .addDisplacementMarker(() -> {})
                                .splineToConstantHeading(new Vector2d(0, 35), Math.toRadians(180))

                                .lineTo(new Vector2d(-15, 35))
                                .splineToConstantHeading(new Vector2d(-56, 35.5), Math.toRadians(180))
                                .waitSeconds(1)

                                .lineTo(new Vector2d(-55, 35.5))
                                .splineToConstantHeading(new Vector2d(-15, 35), Math.toRadians(0))
                                .lineTo(new Vector2d(0, 35))
                                .splineToConstantHeading(new Vector2d(54.6, 36), Math.toRadians(0))
                                .waitSeconds(.2)

                                //Cycle 2
                                .lineTo(new Vector2d(54, 36))
                                .splineToConstantHeading(new Vector2d(0, 35), Math.toRadians(180))
                                .lineTo(new Vector2d(-15, 35))
                                .splineToConstantHeading(new Vector2d(-56, 35.5), Math.toRadians(180))
                                .waitSeconds(1)

                                .lineTo(new Vector2d(-55, 35.5))
                                .splineToConstantHeading(new Vector2d(-15, 35), Math.toRadians(0))
                                .lineTo(new Vector2d(0, 35))
                                .splineToConstantHeading(new Vector2d(54.6, 36), Math.toRadians(0))
                                .waitSeconds(.2)

                                //Cycle 3
                                .lineTo(new Vector2d(54, 36))
                                .splineToConstantHeading(new Vector2d(0, 35), Math.toRadians(180))
                                .lineTo(new Vector2d(-15, 35))
                                .splineToConstantHeading(new Vector2d(-56, 22.5), Math.toRadians(180))
                                .waitSeconds(1)

                                .lineTo(new Vector2d(-55, 22.5))
                                .splineToConstantHeading(new Vector2d(-15, 35), Math.toRadians(0))
                                .lineTo(new Vector2d(0, 35))
                                .splineToConstantHeading(new Vector2d(54.6, 36), Math.toRadians(0))
                                .waitSeconds(.2)
         */



        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK).setDarkMode(true)
                // Background opacity from 0-1
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
