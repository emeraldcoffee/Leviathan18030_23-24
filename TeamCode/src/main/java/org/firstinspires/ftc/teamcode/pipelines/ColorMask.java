package org.firstinspires.ftc.teamcode.pipelines;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.opencv.core.Core.bitwise_and;
import static org.opencv.core.Core.bitwise_not;
import static org.opencv.core.Core.bitwise_or;
import static org.opencv.core.Core.inRange;
import static org.opencv.imgproc.Imgproc.COLOR_BGR2HSV;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.utils.Converters;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;

public class ColorMask extends OpenCvPipeline {
    /*public enum teamElementPosition {
        LEFT,
        CENTER,
        RIGHT
    }*/

    String alliance;
    String start;

    int leftBound;
    int rightBound;
    static int xCoord;
    static int yCoord;

    //public ColorMask.teamElementPosition pos;

    //Colors that will be used as rectangle border
    private final Scalar
            green = new Scalar(0, 255, 0),
            red = new Scalar(255, 0, 0);

//    Rect leftRectangle = new Rect(100, 300, 80, 180);
//    Rect centerRectangle = new Rect(200, 320, 180, 80);
//    Rect rightRectangle = new Rect(530, 300, 80, 180);
    ArrayList<double[]> frameList;
    static Point contourCoords;

    //these are public static to be tuned in dashboard
    public ColorMask() {
        frameList = new ArrayList<>();
    }

    @Override
    public Mat processFrame(Mat input) {


        Rect crop = new Rect(0, 60, 640, 420);
        input = input.submat(crop);

//        Mat output = input;
//        Mat output = input.adjustROI(120, 480, 0, 640);
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        if (mat.empty()) {
            return input;
        }

        Mat thresh = new Mat();
        Scalar lowLHSV;
        Scalar highLHSV;
        Scalar lowerLHSV;
        Scalar higherLHSV;

        Scalar lowHHSV;
        Scalar highHHSV;
        Scalar lowerHHSV;
        Scalar higherHHSV;

        if (alliance.equals("Blue")) {
            lowLHSV = new Scalar(90, 40, 18);
            highLHSV = new Scalar(140, 255, 180);
            Core.inRange(mat, lowLHSV, highLHSV, thresh);
            lowerLHSV = new Scalar(95, 60, 32);
            higherLHSV = new Scalar(135, 255, 200);
            Core.inRange(mat, lowerLHSV, higherLHSV, thresh);

            /*lowHSV = new Scalar(85, 0, 20);
            highHSV = new Scalar(120, 200, 255);
            Core.inRange(mat, lowHSV, highHSV, thresh);
            lowerHSV = new Scalar(95, 0, 20);
            higherHSV = new Scalar(135, 150, 255);
            Core.inRange(mat, lowerHSV, higherHSV, thresh);*/
        } else if (alliance.equals("Red")) {
            Mat threshLow = new Mat();
            Mat threshHigh = new Mat();
            //how to get both aspects, low and high? if make 2 mats, how combine? see later.
            lowLHSV = new Scalar(0, 40, 18);
            highLHSV = new Scalar(15, 255, 180);
            Core.inRange(mat, lowLHSV, highLHSV, thresh);
            lowerLHSV = new Scalar(0, 60, 32);
            higherLHSV = new Scalar(13, 255, 200);
            Core.inRange(mat, lowerLHSV, higherLHSV, thresh);

//            bitwise_and(threshLow, threshHigh, thresh);

//            lowHHSV = new Scalar(140, 10, 5);
//            highHHSV = new Scalar(180, 255, 255);
//            Core.inRange(mat, lowHHSV, highHHSV, threshHigh);
//            lowerHHSV = new Scalar(145, 15, 10);
//            higherHHSV = new Scalar(180, 255, 255);
//            Core.inRange(mat, lowerHHSV, higherHHSV, threshHigh);
//
//            bitwise_or(threshLow, threshHigh, thresh);


        }

        Mat masked = new Mat();
        Core.bitwise_and(mat, mat, masked, thresh);
        Scalar average = Core.mean(masked, thresh);

        Mat scaledMask = new Mat();
        masked.convertTo(scaledMask, -1, 150 / average.val[1], 0);

        Mat scaledThresh = new Mat();
        Scalar strictLowHSV = new Scalar(0, 10, 0); // 140
        Scalar strictHighHSV = new Scalar(255, 255, 255);
        Core.inRange(scaledMask, strictLowHSV, strictHighHSV, scaledThresh);

        Mat finalMask = new Mat();
        Core.bitwise_and(mat, mat, finalMask, scaledThresh);
        Mat edges = new Mat();
        Imgproc.Canny(scaledThresh, edges, 100, 200);
        Imgproc.Canny(scaledThresh, edges, 100, 200);

        //contours, apply post processing to information
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        //find contours, input scaledThresh because it has hard edges
        Imgproc.findContours(scaledThresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        //list of frames to reduce inconsistency, not too many so that it is still real-time, change the number from 5 if you want

        if (frameList.size() > 2) {
            frameList.remove(0);
        }

//        Mat returner = output;

        //release all the data
//        output.release();
        input.release();
        scaledThresh.copyTo(input);
        scaledThresh.release();
        scaledMask.release();
        mat.release();
        masked.release();
        edges.release();
        thresh.release();
        finalMask.release();

        //change the return to whatever mat you want
        //for example, if I want to look at the lenient thresh:
        // return thresh;
        // note that you must not do thresh.release() if you want to return thresh
        // you also need to release the input if you return thresh(release as much as possible)
//        Mat image = new Mat();

        if (contours.size() > 0) {
            double maxContour = 0;
            MatOfPoint largestContour = contours.get(0);

            for (MatOfPoint contour : contours) {
                if (Imgproc.contourArea(contour) > maxContour) {
                    maxContour = Imgproc.contourArea(contour);
                    largestContour = contour;
                }
            }

            Rect rect = Imgproc.boundingRect(largestContour);
            Scalar red = new Scalar(255, 0, 0);


//            Core.bitwise_and(input, output, image);

            Imgproc.rectangle(input, rect, red);


            xCoord = (int) (rect.x + (rect.width / 2));
            yCoord = (int) (rect.y + (rect.height / 2));

            List<Point> coords = new ArrayList<>();
            Converters.Mat_to_vector_Point(largestContour, coords);
            if (coords.size() > 0)
                contourCoords = coords.get(0);
        }
        Rect centerBox = new Rect(leftBound, 0, rightBound - leftBound, 480);
        Scalar green = new Scalar(44, 235, 28);
        Imgproc.rectangle(input, centerBox, green, 2);

        Imgproc.rectangle(input, crop, green, 2);


        return input;
    }

    public void setAlliance(String a) {
        alliance = a;
    }
    public void setStart(String s) {
        start = s;
        if (start.equals("close")) {
            leftBound = 200;
            rightBound = 390;
        }
        else {
            leftBound = 345;
            rightBound = 535;
        }
    }

    public String getPos() {
//        if (start.equals("close")) {close
            if ((xCoord > 0) && (xCoord <= leftBound)) {
                //pos = teamElementPosition.LEFT;
                return "left";
            } else if (xCoord <= rightBound) {
                //pos = teamElementPosition.CENTER;
                return "center";
            } else {
                //pos = teamElementPosition.RIGHT;
                return "right";
            }
    }
}