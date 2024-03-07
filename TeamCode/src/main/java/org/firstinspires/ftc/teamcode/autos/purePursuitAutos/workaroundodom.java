package org.firstinspires.ftc.teamcode.autos.purePursuitAutos;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.Odometry;

import org.firstinspires.ftc.teamcode.drive.RobotConfig;

public class workaroundodom extends Odometry {
    RobotConfig robotConfig;

    public workaroundodom(Pose2d robotPose, RobotConfig robot) {
        super(robotPose);
        robotConfig = robot;
    }

    public workaroundodom(Pose2d robotPose, double trackWidth) {
        super(robotPose, trackWidth);
    }

    @Override
    public void updatePose(Pose2d newPose) {
        robotConfig.setPoseEstimate(new com.acmerobotics.roadrunner.geometry.Pose2d(newPose.getX(), newPose.getY(), newPose.getHeading()));
    }

    @Override
    public void updatePose() {
        robotConfig.update();
        robotPose = new Pose2d(robotConfig.getPoseEstimate().getX(), robotConfig.getPoseEstimate().getY(), new Rotation2d(robotConfig.getPoseEstimate().getHeading()) );
    }
}
