package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.AutoWayPoints;

public class PassData {
    //Used to pass the position of the robot from one program to another, mainly for auto -> tele
    public static RobotConstants.ParkPosition roadrunnerParkPosition = RobotConstants.ParkPosition.WALL;

    //Pure pursuit autos
    public static AutoWayPoints.StackPosition stackPosition = AutoWayPoints.StackPosition.WALL;
    public static AutoWayPoints.DropPosition dropPosition = AutoWayPoints.DropPosition.WALL;
    public static AutoWayPoints.CrossStackSide crossStackSide = AutoWayPoints.CrossStackSide.WALL;
    public static AutoWayPoints.CrossBackdropSide crossBackdropSide = AutoWayPoints.CrossBackdropSide.WALL;
    public static AutoWayPoints.ParkPosition parkPosition = AutoWayPoints.ParkPosition.CENTER;


    public static Pose2d currentPose = new Pose2d();
    public static boolean slidesInitiated = false;
}
