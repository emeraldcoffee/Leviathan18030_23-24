package org.firstinspires.ftc.teamcode.pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Camera3BoxDetection extends OpenCvPipeline {
    //Defining outputs for the camera
    public enum teamElementPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    private volatile teamElementPosition position;



    //Colors that will be used as rectangle border
    private final Scalar
            green   = new Scalar(0, 255, 0),
            red     = new Scalar(255, 0, 0);

    Rect leftRectangle = new Rect(100, 300, 80, 180);
    Rect centerRectangle = new Rect(200, 320, 180, 80);
    Rect rightRectangle = new Rect(530, 300, 80, 180);

    @Override
    public Mat processFrame(Mat input) {
        //Used to change formats not necessary here
        //Imgproc.cvtColor(input, input, Imgproc.COLOR_YCrCb2RGB);
        double leftValue, centerValue, rightValue, winningValue;

        //Set rectangles to be completely inside of object your detecting when its in that position


        //Getting cropped sections of the camera for comparisons
        Mat leftSquare = input.submat(leftRectangle);
        Mat centerSquare = input.submat(centerRectangle);
        Mat rightSquare = input.submat(rightRectangle);

        //Creating images containing just 1 rgb value (coi 0 = red, 1 = green, 2 = blue)
        Core.extractChannel(leftSquare, leftSquare, 0);
        Core.extractChannel(centerSquare, centerSquare, 0);
        Core.extractChannel(rightSquare, rightSquare, 0);

        Scalar leftAvg = Core.mean(leftSquare), centerAvg = Core.mean(centerSquare), rightAvg = Core.mean(rightSquare);

        leftValue = leftAvg.val[0];
        centerValue = centerAvg.val[0];
        rightValue = rightAvg.val[0];

        //setting winningValue to the value with the most of a pixel color
        winningValue = Math.max(leftValue, Math.max(centerValue, rightValue));

        Mat output = new Mat();
        if (leftValue == winningValue) {
            position = teamElementPosition.LEFT;
            input.copyTo(output);
            Imgproc.rectangle(output, leftRectangle, green, 2);
            Imgproc.rectangle(output, centerRectangle, red, 2);
            Imgproc.rectangle(output, rightRectangle, red, 2);
        } else if (rightValue == winningValue) {
            position = teamElementPosition.RIGHT;
            input.copyTo(output);
            Imgproc.rectangle(output, leftRectangle, red, 2);
            Imgproc.rectangle(output, centerRectangle, red, 2);
            Imgproc.rectangle(output, rightRectangle, green, 2);
        } else {
            position = teamElementPosition.CENTER;
            input.copyTo(output);
            Imgproc.rectangle(output, leftRectangle, red, 2);
            Imgproc.rectangle(output, centerRectangle, green, 2);
            Imgproc.rectangle(output, rightRectangle, red, 2);
        }

        //Returning camera image with colored squares overlayed
        return(output);
    }
    //Method that returns the identified position
    public teamElementPosition getObjectPosition() {return position;}
}
