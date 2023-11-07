package org.firstinspires.ftc.teamcode.pipelines;

import static org.opencv.core.Core.inRange;
import static org.opencv.imgproc.Imgproc.COLOR_BGR2HSV;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorMask extends OpenCvPipeline {

    public enum teamElementPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    private volatile Camera3BoxDetection.teamElementPosition position;

    //Colors that will be used as rectangle border
    private final Scalar
            green   = new Scalar(0, 255, 0),
            red     = new Scalar(255, 0, 0);

    Rect leftRectangle = new Rect(100, 300, 80, 180);
    Rect centerRectangle = new Rect(200, 320, 180, 80);
    Rect rightRectangle = new Rect(530, 300, 80, 180);

    @Override
    public Mat processFrame(Mat input) {
        double leftValue, centerValue, rightValue, winningValue;

        Mat leftSquare = input.submat(leftRectangle);
        Mat centerSquare = input.submat(centerRectangle);
        Mat rightSquare = input.submat(rightRectangle);

        Imgproc.cvtColor(input, leftSquare, COLOR_BGR2HSV);
        Scalar blueLow = new Scalar(100, 150, 0);
        Scalar blueHigh = new Scalar(140, 255, 255);
        Mat leftMask = new Mat();
        inRange(leftSquare, blueLow, blueHigh, leftMask);
        input.setTo(new Scalar(0, 0, 255), leftMask);

        Mat centMask = new Mat();
        inRange(leftSquare, blueLow, blueHigh, centMask);
        input.setTo(new Scalar(0, 0, 255), centMask);

        Mat rightMask = new Mat();
        inRange(leftSquare, blueLow, blueHigh, rightMask);
        input.setTo(new Scalar(0, 0, 255), rightMask);


        return input;
    }
}
