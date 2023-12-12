package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.dfrobot.HuskyLens;


public class huskyLensDetection {

    int xcoord = 0;

    public String getPos(HuskyLens h) {
        for (HuskyLens.Block b : h.blocks()) {
            int objID = b.id;
            if ((objID == 2) || (objID == 3)) {
                xcoord = b.x;
            }
        }
        if ((xcoord > 0) && (xcoord <= 213)) {
            //pos = teamElementPosition.LEFT;
            return "left";
        } else if (xcoord <= 450) {
            //pos = teamElementPosition.CENTER;
            return "center";
        } else {
            //pos = teamElementPosition.RIGHT;
            return "right";

        }
    }
}
