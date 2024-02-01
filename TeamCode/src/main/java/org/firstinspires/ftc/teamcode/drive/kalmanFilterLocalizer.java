package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.video.KalmanFilter;

import java.util.ArrayList;
import java.util.List;
import java.util.logging.Handler;

@Config
public class kalmanFilterLocalizer {
    boolean backDropRelocalization = false;

    public void kalmanFiler(HardwareMap hardwareMap) {

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        StandardTrackingWheelLocalizer odomLocalizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);
    }


    public void update() {

    }
    public void backDropReloclization(boolean use) {
        backDropRelocalization = use;
    }
}
