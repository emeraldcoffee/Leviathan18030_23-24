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

}
