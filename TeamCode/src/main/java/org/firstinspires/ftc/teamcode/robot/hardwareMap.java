package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.Arrays;
import java.util.List;

public class hardwareMap {
    //Encoder names
    public Encoder encoder1;
    //Motor names (drive train motors are in drive/SampleMecanumDrive)
    public DcMotorEx climbMotor, intakeMotor, transferMotor;
    //Servo names
    public Servo dropServo;
    //Camera name
    public OpenCvCamera frontCamera;

    private List<DcMotorEx> motors;

    private ElapsedTime period = new ElapsedTime();

    public void init(HardwareMap hwMap) {

        //Setting up camera
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        frontCamera = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "camera"), cameraMonitorViewId);

        //Mapping encoder
        //encoder1 = new Encoder(hwMap.get(DcMotorEx.class, "leftRear"));

        //Optionally reverse the encoders with encoder1.setDirection(Encoder.Direction.REVERSE);


        //Mapping motors
        climbMotor = hwMap.get(DcMotorEx.class, "climbMotor");
        intakeMotor = hwMap.get(DcMotorEx.class, "intakeMotor");
        transferMotor = hwMap.get(DcMotorEx.class, "transferMotor");

        //Creating list of motors to setup
        motors = Arrays.asList(climbMotor, intakeMotor, transferMotor);

        //Configuring motors
        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        //Optionally reverse them with: motor1.setDirection(DcMotorSimple.Direction.REVERSE);

        //Mapping Servos
        dropServo = hwMap.servo.get("dropServo");

        //Optionally reverse them with: servo1.setDirection(Servo.Direction.REVERSE);


        //Reads all sensor data at once and saves it, instead of doing multiple individual reads. It talks just as long to read all non-12c senors as it takes to read one
        for (LynxModule module : hwMap.getAll(LynxModule.class))
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

    }
}
