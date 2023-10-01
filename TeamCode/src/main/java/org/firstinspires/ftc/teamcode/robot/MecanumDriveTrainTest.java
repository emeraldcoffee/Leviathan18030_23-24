package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class MecanumDriveTrainTest {

    HardwareMap hwMap;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor rearLeft;
    DcMotor rearRight;
    //DcMotor intake
    //DcMotor lift;
    //DcMotor hang;
    //Servo outtake;
    IMU imu;

    public void init (HardwareMap hwMap) {
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        rearLeft = hwMap.get(DcMotor.class, "rearLeft");
        rearRight = hwMap.get(DcMotor.class, "rearRight");
        //lift = hwMap.get(DcMotor.class, "lift");
        //outtake = hwMap.get(Servo.class, "outtake");

        setL(frontLeft);
        setR(frontRight);
        setL(rearLeft);
        setR(rearRight);

        /*setL(lift);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);*/

        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, // configure properly
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
    }

    public void setR (DcMotor motor) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setL (DcMotor motor) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Double getExternalHeadingVelocity() {
        return (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).yRotationRate;
    }
}
