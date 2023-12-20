package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.dfrobot.HuskyLens;


public class huskyLensDetection {

    // add in method to set which huskylens is initiated (right/left) based on what the alliance is, called in autonomous codes

    int id = 0;

    public String getPos(HuskyLens h) {
        h.selectAlgorithm(HuskyLens.Algorithm.OBJECT_CLASSIFICATION);

        for (HuskyLens.Block b : h.blocks()) {
            id = b.id;
        }

        if (id == 1) {
            //pos = teamElementPosition.LEFT;
            return "left";
        } else if (id == 2) {
            //pos = teamElementPosition.CENTER;
            return "center";
        } else {
            //pos = teamElementPosition.RIGHT;
            return "right";
        }

    }
}
