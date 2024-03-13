package org.firstinspires.ftc.teamcode.pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.utils.Converters;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class YellowPosDetection extends OpenCvPipeline {

    String alliance;

    public YellowPosDetection() {

    }

    @Override
    public Mat processFrame(Mat input) {

        int xCoord;
        int yCoord;

        Mat output = input;
        Mat mat = new Mat();

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Mat thresh = new Mat();

        Scalar lowHSV = new Scalar(15, 30, 20);
        Scalar highHSV = new Scalar(35, 255, 255);
        Core.inRange(mat, lowHSV, highHSV, thresh);
        Scalar lowerHSV = new Scalar(18, 30, 20);
        Scalar higherHSV = new Scalar(32, 255, 255);
        Core.inRange(mat, lowerHSV, higherHSV, thresh);

        Mat masked = new Mat();
        Core.bitwise_and(mat, mat, masked, thresh);
        Scalar average = Core.mean(masked, thresh);

        Mat scaledMask = new Mat();
        masked.convertTo(scaledMask, -1, 150/average.val[1], 0);

        Mat scaledThresh = new Mat();
        Scalar strictLowHSV = new Scalar(0, 10, 0); // 140
        Scalar strictHighHSV = new Scalar(255, 255, 255);
        Core.inRange(scaledMask, strictLowHSV, strictHighHSV, scaledThresh);

        Mat finalMask = new Mat();
        Core.bitwise_and(mat, mat, finalMask, scaledThresh);
        Mat edges = new Mat();
        Imgproc.Canny(scaledThresh, edges, 100, 200);
        Imgproc.Canny(scaledThresh, edges, 100, 200);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(scaledThresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        input.release();
        scaledThresh.copyTo(input);
        scaledThresh.release();
        scaledMask.release();
        mat.release();
        masked.release();
        edges.release();
        thresh.release();
        finalMask.release();

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

            Imgproc.rectangle(output, rect, red);

            xCoord = (int) (rect.x + (rect.width / 2));
            yCoord = (int) (rect.y + (rect.height / 2));

            List<Point> coords = new ArrayList<>();
            Converters.Mat_to_vector_Point(largestContour, coords);
        }
        return output;
    }
}
