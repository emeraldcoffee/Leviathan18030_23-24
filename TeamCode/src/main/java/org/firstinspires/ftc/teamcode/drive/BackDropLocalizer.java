package org.firstinspires.ftc.teamcode.drive;

import static java.lang.Math.abs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.i2cDrivers.UltraSonic;
import org.firstinspires.ftc.teamcode.robot.RobotMethods;


public class BackDropLocalizer extends TwoDistanceLocalizer{

    Pose2d poseEstimate = new Pose2d();

    public enum RelBackdropPose{
        FAR,
        LEFT,
        CENTER,
        RIGHT,
    }


    private DistanceSensor leftDistanceSensor, rightDistanceSensor;
    private UltraSonic leftUltrasonic, rightUltrasonic;

    private TouchSensor leftReader, rightReader;

    private volatile double leftDistance, rightDistance;


    boolean backDropRelocalization = false;


    public BackDropLocalizer(HardwareMap hardwareMap) {
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "frontLeftDistanceSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "frontRightDistanceSensor");

        leftUltrasonic = hardwareMap.get(UltraSonic.class, "leftUltra");
        rightUltrasonic = hardwareMap.get(UltraSonic.class, "rightUltra");

        leftReader = hardwareMap.get(TouchSensor.class, "leftReader");
        rightReader = hardwareMap.get(TouchSensor.class, "rightReader");
    }

//    Runnable updateLeft = () -> {
//        leftDistance = -leftDistanceSensor.getDistance(DistanceUnit.INCH);
//    };
//    Runnable updateRight = () -> {
//        rightDistance = -rightDistanceSensor.getDistance(DistanceUnit.INCH);
//    };

    public void update() {
        leftDistance = -leftDistanceSensor.getDistance(DistanceUnit.INCH);
        rightDistance = -rightDistanceSensor.getDistance(DistanceUnit.INCH);


        double heading = -Math.atan((leftDistance-rightDistance)/7.673);
        double lateralDistance = ((leftDistance+rightDistance)/2-6.524)*Math.cos(heading);
//should read 51.7
        poseEstimate = new Pose2d(lateralDistance+61.3, poseEstimate.getY(), heading);
    }

    public Pose2d getPoseEstimate() {
        return new Pose2d(poseEstimate.getX(), 0, poseEstimate.getHeading());
    }

    public Pose2d getPoseEstimate(Pose2d pose) {
        return new Pose2d(poseEstimate.getX(), pose.getY(), poseEstimate.getHeading());
    }

    public double getLeftUltrasonic() {
        return leftUltrasonic.reportRangeReadingIN()*Math.cos(poseEstimate.getHeading());
    }

    public double getRightUltrasonic() {
        return rightUltrasonic.reportRangeReadingIN()*Math.cos(poseEstimate.getHeading());
    }

    public void takeLeftReading() {
        leftUltrasonic.rangeReading();
    }

    public void takeRightReading() {
        rightUltrasonic.rangeReading();
    }

    public boolean isLeftReading() {
        return !leftReader.isPressed();
    }

    public boolean isRightReading() {
        return !rightReader.isPressed();
    }

    public Pose2d getPoseEstimateLeft() {
        return new Pose2d(poseEstimate.getX(), 61.8-getLeftUltrasonic(), poseEstimate.getHeading());
    }

    public Pose2d getPoseEstimateRight() {
        return new Pose2d(poseEstimate.getX(), getRightUltrasonic()-63, poseEstimate.getHeading());
    }

    public boolean isInRange(Pose2d inputPose) {
        return abs(poseEstimate.getX() - inputPose.getX()) < 1 &&
                abs(RobotMethods.angleDifferenceRad(poseEstimate.getHeading(), inputPose.getHeading())) < Math.toRadians(5);
    }

    public boolean isInRange(Pose2d inputPose, double XrangeVal, double headingRangeVal) {
        return abs(poseEstimate.getX() - inputPose.getX()) < XrangeVal &&
                abs(RobotMethods.angleDifferenceRad(poseEstimate.getHeading(), inputPose.getHeading())) < headingRangeVal;
    }

//    public Enum getRelBackdropPose(Pose2d inputPose) {
//        //The bigger the number the further the sensors are reading
//        //Bigger numbers signify that it is less infront of the backdrop
//        double headingDiff = poseEstimate.getHeading()-inputPose.getHeading();
//
//        if (headingDiff<Math.toRadians(-5)) {
//            return RelBackdropPose.LEFT;
//        } else if (headingDiff>Math.toRadians(5)) {
//            return RelBackdropPose.RIGHT;
//        } else if (inputPose.getX()-poseEstimate.getX()<2) {
//            return RelBackdropPose.CENTER;
//        } else return RelBackdropPose.FAR;
//    }

}
