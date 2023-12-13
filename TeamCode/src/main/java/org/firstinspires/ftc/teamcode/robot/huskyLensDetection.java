package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.dfrobot.HuskyLens;


public class huskyLensDetection {

    int xcoord = 0;
    int objID = 0;
    public String getPos(HuskyLens h) {
        h.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);

        for (HuskyLens.Block b : h.blocks()) {
            xcoord = b.x;
        }

        if ((xcoord > 0) && (xcoord <= 80)) {
            //pos = teamElementPosition.LEFT;
            return "left";
        } else if (xcoord <= 240) {
            //pos = teamElementPosition.CENTER;
            return "center";
        } else {
            //pos = teamElementPosition.RIGHT;
            return "right";
        }

    }
}
