package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class PassData {
    //Used to pass the position of the robot from one program to another, mainly for auto -> tele
    public static Pose2d currentPose = new Pose2d();
    public static boolean slidesInitiated = false;
}
