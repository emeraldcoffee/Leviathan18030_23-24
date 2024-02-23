package com.example.meepmeeptesting;

public class RobotConstants {

    public static  double leftIn = 0.5, rightIn = .5;

    public static enum ParkPosition {
        WALL(-45, 45),
        CENTER(-12, 12);

        public final double red, blue;

        ParkPosition(double red, double blue) {
            this.red = red;
            this.blue = blue;
        }
    }

}
