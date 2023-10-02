package org.firstinspires.ftc.teamcode.pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class testPipeline extends OpenCvPipeline {

    private double
            leftVal,
            centVal,
            rightVal,
            winVal;

    private Rect
            leftRect = new Rect(100, 100, 60, 120),
            centRect = new Rect (150, 150, 60, 120),
            rightRect = new Rect(200, 200, 60, 120);

    public enum propPos {
        LEFT,
        RIGHT,
        CENT
    }

    private volatile propPos pos;

    private final Scalar
        green = new Scalar(0, 255, 0),
        red = new Scalar (255, 0, 0);

    @Override
    public Mat processFrame(Mat input) {

        Mat leftSquare = input.submat(leftRect);
        Mat centSquare = input.submat(centRect);
        Mat rightSquare = input.submat(rightRect);
        Mat output = new Mat();

        Core.extractChannel(leftSquare, leftSquare, 0);
        Core.extractChannel(centSquare, centSquare, 0);
        Core.extractChannel(rightSquare, rightSquare, 0);

        Scalar leftAvg = Core.mean(leftSquare);
        Scalar centAvg = Core.mean(centSquare);
        Scalar rightAvg = Core.mean(rightSquare);

        leftVal = leftAvg.val[0];
        centVal = centAvg.val[0];
        rightVal = rightAvg.val[0];

        winVal = Math.max(leftVal, (Math.max(centVal, rightVal)));

        if (winVal == leftVal) {
            pos = propPos.LEFT;
            input.copyTo(output);
            Imgproc.rectangle(output, leftRect, green, 2);
            Imgproc.rectangle(output, centRect, red, 2);
            Imgproc.rectangle(output, rightRect, red, 2);
        }
        else if (winVal == rightVal) {
            pos = propPos.RIGHT;
            Imgproc.rectangle(output, leftRect, red, 2);
            Imgproc.rectangle(output, centRect, red, 2);
            Imgproc.rectangle(output, rightRect, green, 2);
        }
        else {
            pos = propPos.CENT;
            Imgproc.rectangle(output, leftRect, red, 2);
            Imgproc.rectangle(output, centRect, green, 2);
            Imgproc.rectangle(output, rightRect, red, 2);
        }

        return output;
    }

    public propPos getPos() {
        return pos;
    }
}
