package org.firstinspires.ftc.teamcode.autos;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.ColorMask;
import org.firstinspires.ftc.teamcode.robot.HwMap;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.huskyLensDetection;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "HuskyAutoTest")
public class HuskyAutoTest extends LinearOpMode {
    int targetSlidePos = RobotConstants.slideAuto;

    double slideI = 0;

    String pos = "";
    huskyLensDetection h = new huskyLensDetection();

    enum Camera {
        WAIT,
        SAVE,
        FINISHED
    }

    //fCamRedCloseAuto.Camera camera = fCamRedCloseAuto.Camera.WAIT;

    @Override
    public void runOpMode() throws InterruptedException {
        //SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);
        //ColorMask pipeline = new ColorMask();
        //HwMap robot = new HwMap();
        //robot.init(hardwareMap);

        //HuskyLens huskylens = robot.rightHusky;
        HuskyLens huskyLens = hardwareMap.get(HuskyLens.class, "rightHusky");


        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            //driveTrain.update();
            pos = h.getPos(huskyLens);

            telemetry.addData("position", pos);
            telemetry.update();
        }

    }
}