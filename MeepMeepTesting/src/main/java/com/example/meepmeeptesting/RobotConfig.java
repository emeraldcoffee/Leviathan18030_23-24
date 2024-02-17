package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import java.util.Arrays;

public class RobotConfig {
    public enum CurrentIMU {
        expansionIMU,
        controlIMU,
        noIMU;
    }
    CurrentIMU currentIMU = CurrentIMU.expansionIMU;

    motor intakeMotor = new motor();
    motor transferMotor = new motor();

    servo leftPixelServo = new servo();
    servo rightPixelServo = new servo();

    public enum StackArm {
        GUIDE(0.6),
        FAR_OUT(0),
        OUT(.15),
        IN(.3);

        public final double position;
        StackArm(double position) {this.position = position;}
    }

    public enum Dropper {
        CLOSED(.545),
        PARTIAL(.6275),
        OPEN(.71);

        public final double position;
        Dropper(double position) {this.position = position;}
    }

    public enum SlideHeight {
        BOTTOM(0),
        PRELOAD_DROP(11.5),
        LOW(15),
        MEDIUM(21),
        HIGH(30);

        public final double height;
        SlideHeight(double height) {this.height = height;}
    }

    public void stackHold(boolean thing) {

    }

    public void stackArm(StackArm x) {

    }

    public void dropper(Dropper x) {

    }

    public void setTargetSlidePos(SlideHeight x) {

    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

}
