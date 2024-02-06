package org.firstinspires.ftc.teamcode.drive;

import static java.lang.Math.abs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class RearWallLocalizer extends TwoDistanceLocalizer{
    Pose2d poseEstimate = new Pose2d();

    private DistanceSensor leftDistanceSensor, rightDistanceSensor;

    private volatile double leftDistance, rightDistance;

    boolean backDropRelocalization = false;


    public RearWallLocalizer(HardwareMap hardwareMap) {
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "backLeftDistanceSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "backRightDistanceSensor");

    }

    Runnable updateLeft = () -> {
        leftDistance = leftDistanceSensor.getDistance(DistanceUnit.INCH);
    };
    Runnable updateRight = () -> {
        rightDistance = rightDistanceSensor.getDistance(DistanceUnit.INCH);
    };

    public void update() {
        updateLeft.run();
        updateRight.run();

        double heading = -Math.tan((leftDistance-rightDistance)/10.03);
        double lateralDistance = ((leftDistance+rightDistance)/2+9.74)*Math.cos(heading);

        poseEstimate = new Pose2d(lateralDistance-70, poseEstimate.getY(), heading);
    }

    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    public boolean isInRange(Pose2d inputPose) {
        return abs(poseEstimate.getX() - inputPose.getX()) < 1 && abs(poseEstimate.getHeading() - inputPose.getHeading()) < 1;
    }

    public boolean isInRange(Pose2d inputPose, double rangeVal) {
        return abs(poseEstimate.getX() - inputPose.getX()) < rangeVal && abs(poseEstimate.getHeading() - inputPose.getHeading()) < rangeVal;
    }



}
