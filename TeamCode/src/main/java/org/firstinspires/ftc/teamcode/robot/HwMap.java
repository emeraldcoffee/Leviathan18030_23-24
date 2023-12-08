package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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

public class HwMap {


    //Encoder names
    public Encoder liftEncoder;

    public ColorSensor outtakeColorSensor;

    public DistanceSensor intakeDistanceSensor;
    //Motor names (drive train motors are in drive/SampleMecanumDrive)

    /*
        hi celina :)
        Control Hub
        0: slideMotor, left encoder
        1: climbMotor
        2: leftFront
        3: leftRight


        Expansion Hub
        0: rightFront (guess), right encoder
        1: rightRear, front encoder
        2: intakeMotor
        3: tranferMotor

     */

    public DcMotorEx slideMotor, climbMotor, intakeMotor, transferMotor, liftEncoderMotor;
    //Servo names
    public Servo dropServo, leftServo, rightServo, droneServo;

    //Camera name
    public OpenCvCamera webcam, webcamR;
    public WebcamName frontCamera;

    private List<DcMotorEx> motors;

    private ElapsedTime period = new ElapsedTime();

    Camera3BoxDetection camPipe;

    public void init(HardwareMap hwMap) {

        //Setting up camera
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
        //webcamR = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "cameraR"), cameraMonitorViewId);
        frontCamera = hwMap.get(WebcamName.class, "camera");


        //Mapping encoder
        liftEncoder = new Encoder(hwMap.get(DcMotorEx.class, "slideMotor"));

        //lift encoder was set to go to the climbMotor config/motor, i'm not sure which one is the correct wire and
        // i'm lowkey scared of screwing it up, hope judging went well tho! have fun for the rest of today <3 -selena
        //liftEncoder.setDirection(Encoder.Direction.REVERSE);

        outtakeColorSensor = hwMap.get(ColorSensor.class, "outtakeColor");

        intakeDistanceSensor = hwMap.get(DistanceSensor.class, "intakeDistance");

        //Mapping motors
        climbMotor = hwMap.get(DcMotorEx.class, "climbMotor");
        intakeMotor = hwMap.get(DcMotorEx.class, "intakeMotor");
        transferMotor = hwMap.get(DcMotorEx.class, "transferMotor");
        slideMotor = hwMap.get(DcMotorEx.class, "slideMotor");

        //liftEncoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Creating list of motors to setup
        motors = Arrays.asList(climbMotor, intakeMotor, slideMotor, transferMotor);

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
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        climbMotor.setTargetPosition(0);
        climbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climbMotor.setPower(1);

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Mapping Servos
        dropServo = hwMap.servo.get("dropServo");
        leftServo = hwMap.servo.get("leftServo");
        rightServo = hwMap.servo.get("rightServo");
        droneServo = hwMap.servo.get("droneServo");



        //Optionally reverse them with: servo1.setDirection(Servo.Direction.REVERSE);


        //Reads all sensor data at once and saves it, instead of doing multiple individual reads. It talks just as long to read all non-12c senors as it takes to read one
        for (LynxModule module : hwMap.getAll(LynxModule.class))
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

    }
}
