package org.firstinspires.ftc.teamcode.pipelines;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.opencv.core.Core.inRange;
import static org.opencv.imgproc.Imgproc.COLOR_BGR2HSV;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ColorMask extends OpenCvPipeline {

    public enum teamElementPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    private volatile ColorMask.teamElementPosition position;

    //Colors that will be used as rectangle border
    private final Scalar
            green   = new Scalar(0, 255, 0),
            red     = new Scalar(255, 0, 0);

    Rect leftRectangle = new Rect(100, 300, 80, 180);
    Rect centerRectangle = new Rect(200, 320, 180, 80);
    Rect rightRectangle = new Rect(530, 300, 80, 180);

    @Override
    public Mat processFrame(Mat input) {
        double leftValue, centerValue, rightValue, winVal;
        Mat output = new Mat();

        try {
            Mat leftSquare = input.submat(leftRectangle);
            Mat centerSquare = input.submat(centerRectangle);
            Mat rightSquare = input.submat(rightRectangle);


            Imgproc.cvtColor(input, leftSquare, COLOR_BGR2HSV);
            Scalar blueLow = new Scalar(100, 150, 0);
            Scalar blueHigh = new Scalar(140, 255, 255);
            Scalar contourColor = new Scalar(255, 255, 255);
            List<MatOfPoint> contours = new ArrayList<>();


            Mat leftMask = new Mat();
            Mat leftContoursMat = new Mat();
            inRange(leftSquare, blueLow, blueHigh, leftMask);
            input.setTo(new Scalar(0, 0, 255), leftMask);
            Imgproc.findContours(leftMask, contours, leftContoursMat, 1, 2);
            double leftArea = Imgproc.contourArea(leftContoursMat);

            Mat centContoursMat = new Mat();
            Mat centMask = new Mat();
            inRange(leftSquare, blueLow, blueHigh, centMask);
            input.setTo(new Scalar(0, 0, 255), centMask);
            Imgproc.findContours(centMask, contours, centContoursMat, 1, 2);
            double centArea = Imgproc.contourArea(centContoursMat);

            Mat rightContoursMat = new Mat();
            Mat rightMask = new Mat();
            inRange(leftSquare, blueLow, blueHigh, rightMask);
            input.setTo(new Scalar(0, 0, 255), rightMask);
            Imgproc.findContours(rightMask, contours, rightContoursMat, 1, 2);
            double rightArea = Imgproc.contourArea(rightContoursMat);

            winVal = Math.max(leftArea, Math.max(centArea, rightArea));
            if (winVal == leftArea) {
                input.copyTo(output);
                position = teamElementPosition.LEFT;
                Imgproc.rectangle(output, leftRectangle, green, 2);
                Imgproc.rectangle(output, centerRectangle, red, 2);
                Imgproc.rectangle(output, rightRectangle, red, 2);
            }
            if (winVal == centArea) {
                input.copyTo(output);
                position = teamElementPosition.CENTER;
                Imgproc.rectangle(output, leftRectangle, red, 2);
                Imgproc.rectangle(output, centerRectangle, green, 2);
                Imgproc.rectangle(output, rightRectangle, red, 2);
            }
            if (winVal == rightArea) {
                input.copyTo(output);
                position = teamElementPosition.RIGHT;
                Imgproc.rectangle(output, leftRectangle, red, 2);
                Imgproc.rectangle(output, centerRectangle, red, 2);
                Imgproc.rectangle(output, rightRectangle, green, 2);
            }
        }
        catch (Error e) {
            telemetry.addData("Error", e);
        }
    return output;
    }
}
