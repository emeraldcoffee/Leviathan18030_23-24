package com.example.meepmeeptesting;

public class RobotConfig {
    public enum CurrentIMU {
        expansionIMU,
        controlIMU,
        noIMU;
    }
    CurrentIMU currentIMU = CurrentIMU.expansionIMU;

    motor intakeMotor = new motor();
    motor transferMotor = new motor();

    public enum StackArm {
        GUIDE(0.6),
        FAR_OUT(0),
        OUT(.15),
        IN(.3);

        public final double position;
        StackArm(double position) {this.position = position;}
    }

    public void stackHold(boolean thing) {

    }

//    public void setTargetSlidePos()
}
