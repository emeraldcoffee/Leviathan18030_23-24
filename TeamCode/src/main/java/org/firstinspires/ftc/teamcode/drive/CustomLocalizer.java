package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

@Config
public class CustomLocalizer implements Localizer, CustomLocalizerImp {
    Pose2d poseEstimate = new Pose2d(0,0,0);
    StandardTrackingWheelLocalizer odomLocalizer;
    BackDropLocalizer backDropLocalizer;
    RearWallLocalizer rearWallLocalizer;
public CustomLocalizer(HardwareMap hardwareMap,  List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {
    odomLocalizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);
    odomLocalizer.setPoseEstimate(poseEstimate);

    backDropLocalizer = new BackDropLocalizer(hardwareMap);
//    rearWallLocalizer = new RearWallLocalizer(hardwareMap);
}



    public void update() {
        odomLocalizer.update();
        poseEstimate = odomLocalizer.getPoseEstimate();
    }

    public void updateBackdrop() {
        backDropLocalizer.update();
        setPoseEstimate(new Pose2d(backDropLocalizer.getPoseEstimate().getX(), poseEstimate.getY(), backDropLocalizer.getPoseEstimate().getHeading()));

    }


    public void compensatedUpdateBackdrop() {
        backDropLocalizer.update();
        setPoseEstimate(new Pose2d(poseEstimate.getX()*.5+backDropLocalizer.getPoseEstimate().getX()*.5, poseEstimate.getY(), backDropLocalizer.getPoseEstimate().getHeading()));

    }

    public void smartUpdateBackdrop() {
        backDropLocalizer.update();
        if (backDropLocalizer.isInRange(poseEstimate)) {
            setPoseEstimate(new Pose2d(backDropLocalizer.getPoseEstimate().getX(), poseEstimate.getY(), backDropLocalizer.getPoseEstimate().getHeading()));
        }
    }

    public void smartCompensatedUpdateBackdrop() {
        backDropLocalizer.update();
        if (backDropLocalizer.isInRange(poseEstimate)) {
            setPoseEstimate(new Pose2d(poseEstimate.getX()*.5+backDropLocalizer.getPoseEstimate().getX()*.5, poseEstimate.getY(), backDropLocalizer.getPoseEstimate().getHeading()));
        }
    }

//    public void

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        poseEstimate = pose2d;
        odomLocalizer.setPoseEstimate(poseEstimate);
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return odomLocalizer.getPoseVelocity();
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }
}
