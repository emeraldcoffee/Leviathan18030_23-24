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
public class CustomLocalizer implements Localizer {
    Pose2d poseEstimate = new Pose2d(0,0,0);
    StandardTrackingWheelLocalizer odomLocalizer;
    BackDropLocalizer backDropLocalizer;
    RearWallLocalizer rearWallLocalizer;

    public void localizers(HardwareMap hardwareMap) {

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        odomLocalizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);
        odomLocalizer.setPoseEstimate(poseEstimate);

        backDropLocalizer = new BackDropLocalizer(hardwareMap);
        rearWallLocalizer = new RearWallLocalizer(hardwareMap);



    }


    public void update() {
        odomLocalizer.update();
        backDropLocalizer.update();
        rearWallLocalizer.update();
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
