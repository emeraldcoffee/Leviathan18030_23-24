package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class BackDropLocalizer extends TwoDistanceLocalizer{
    Pose2d poseEstimate = new Pose2d();

    private DistanceSensor leftDistanceSensor, rightDistanceSensor;

    private volatile double leftDistance, rightDistance;


    boolean backDropRelocalization = false;


    public BackDropLocalizer(HardwareMap hardwareMap) {
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "frontLeftDistanceSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "frontRightDistanceSensor");

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

        poseEstimate = new Pose2d(poseEstimate.getX(), lateralDistance, heading);
    }

    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }


}
