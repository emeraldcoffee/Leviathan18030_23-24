package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pipelines.Camera3BoxDetection;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.Arrays;
import java.util.List;

public class hardwareMap {

    //Encoder names
    public Encoder liftEncoder;
    //Motor names (drive train motors are in drive/SampleMecanumDrive)

    /*
        front is towards the intake, back is towards the outtake
        horizontal directions are from the back of the robot

        control hub
        motor port 0:
            encoder port 0: liftEncoder
        motor port 1: liftMotor
            encoder port 1: leftEncoder
        motor port 2: backLeft
            encoder port 2:
        motor port 3: frontLeft
            encoder port 3: centEncoder

        expansion hub
        motor port 0: frontRight
            encoder port 0:
        motor port 1: backRight
            encoder port 1:
        motor port 2:
            encoder port 2:
        motor port 3:
            encoder port 3: rightEncoder
     */

    public DcMotorEx liftMotor, climbMotor, intakeMotor, transferMotor;
    //Servo names
    public Servo dropServo;

    //Camera name
    public OpenCvCamera webcam;

    private List<DcMotorEx> motors;

    private ElapsedTime period = new ElapsedTime();

    Camera3BoxDetection camPipe;

    public void init(HardwareMap hwMap) {

        //Setting up camera
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "camera"), cameraMonitorViewId);


        //Mapping encoder
        liftEncoder = new Encoder(hwMap.get(DcMotorEx.class, "liftMotor"));
        //liftEncoder.setDirection(Encoder.Direction.REVERSE);

        //Optionally reverse the encoders with encoder1.setDirection(Encoder.Direction.REVERSE);


        //Mapping motors
        climbMotor = hwMap.get(DcMotorEx.class, "climbMotor");
        intakeMotor = hwMap.get(DcMotorEx.class, "intakeMotor");
        transferMotor = hwMap.get(DcMotorEx.class, "transferMotor");
        liftMotor = hwMap.get(DcMotorEx.class, "liftMotor");

        //Creating list of motors to setup
        motors = Arrays.asList(climbMotor, intakeMotor, liftMotor, transferMotor);

        //Configuring motors
        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        //Optionally reverse them with: motor1.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //Mapping Servos
        dropServo = hwMap.servo.get("dropServo");

        //Optionally reverse them with: servo1.setDirection(Servo.Direction.REVERSE);


        //Reads all sensor data at once and saves it, instead of doing multiple individual reads. It talks just as long to read all non-12c senors as it takes to read one
        for (LynxModule module : hwMap.getAll(LynxModule.class))
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

    }
}
