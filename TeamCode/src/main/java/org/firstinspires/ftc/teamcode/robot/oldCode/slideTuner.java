package org.firstinspires.ftc.teamcode.robot.oldCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.HwMap;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.RobotMethods;

@Disabled
@Config
@Autonomous
public class slideTuner extends LinearOpMode {

    public static double  cycleTime = 5;
    double slideI = 0;
    public static double PID_P = 2, PID_I = .01, PID_D = .001;
    PIDCoefficients slidePIDVals = new PIDCoefficients(PID_P / 8192, PID_I / 8192, PID_D / 8192);

    enum CycleStages {
        UP,
        DOWN
    }

    CycleStages cycleStages = CycleStages.UP;

    @Override
    public void runOpMode() throws InterruptedException {
        HwMap robot = new HwMap();
        robot.init(hardwareMap);

        ElapsedTime cycleTimer = new ElapsedTime();

        RobotMethods robotMethods = new RobotMethods();

        waitForStart();

        cycleTimer.reset();
        robotMethods.setTargetPos(robot.liftEncoder.getCurrentPosition(), RobotConstants.slideLow);

        while (opModeIsActive() && !isStopRequested()) {
            switch (cycleStages) {
                case UP:
                    if (cycleTimer.seconds()> cycleTime/2) {
                        robotMethods.setTargetPos(RobotConstants.slideBottom, RobotConstants.slideTop);
                        cycleStages = CycleStages.DOWN;
                    }
                    break;
                case DOWN:
                    if (cycleTimer.seconds() > cycleTime) {
                        robotMethods.setTargetPos(RobotConstants.slideTop, RobotConstants.slideBottom);
                        cycleTimer.reset();
                        cycleStages = CycleStages.UP;
                    }
                    break;
            }


            double slideVelo = robot.liftEncoder.getCorrectedVelocity();
            int slideCurPos = robot.liftEncoder.getCurrentPosition();

            double distRemain = robotMethods.slidesUpdate() - slideCurPos;

            slideI += distRemain * slidePIDVals.i;

            //robot.liftMotor.setPower((distRemain * slidePIDVals.p) + slideI + (slideVelo * slidePIDVals.d));
        }
        robot.slideMotor.setPower(0);


    }
}
