package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.PassData;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

@Config
public class RobotConfig {
    // The lateral distance between the left and right odometers
    // is called the trackwidth. This is very important for
    // determining angle for turning approximations
    public static final double LATERAL_DISTANCE = 10.256;

    // Center wheel offset is the distance between the
    // center of rotation of the robot and the center odometer.
    // This is to correct for the error that might occur when turning.
    // A negative offset means the odometer is closer to the back,
    // while a positive offset means it is closer to the front.
    public static final double CENTER_WHEEL_OFFSET = 2.977;

    public static final double WHEEL_DIAMETER = 1.38;
    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 8192;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    public MecanumDrive driveTrain;

    //Hardware Components
    public DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public List<DcMotorEx> driveTrainMotors;

    public DcMotorEx slideMotor, climbMotor, intakeMotor, transferMotor;
    public List<DcMotorEx> allMotors;

    public double targetSlidePos = 0, prevError = 0, prevTime = 0;

    public Encoder slideEncoder;
    public final double slideToInches = 1.6 * Math.PI / 8192;

    public Servo dropServo, leftPixelServo, rightPixelServo, droneServo, leftStackServo, rightStackServo, stackHoldServo;

    List<LynxModule> allHubs;

    public OdometrySubsystem odometry;

    //100.324 100.478 (.996)
    public static double LEFT_MULTIPLIER = .998342;

//99.986 100 (1)
    public static double RIGHT_MULTIPLIER = .99911079;
    //98.816 99.197
    public static double Y_MULTIPLIER = 1.01;

    ElapsedTime timer = new ElapsedTime();

    enum StackArm {
        GUIDE(0.6),
        OUT(.15),
        IN(.3);

        public final double position;
        StackArm(double position) {this.position = position;}
    }

    enum SlideHeight {
        BOTTOM(0),
        PRELOAD_DROP(11.5),
        LOW(15),
        MEDIUM(21),
        HIGH(30);

        public final double height;
        SlideHeight(double height) {this.height = height;}
    }

    enum Dropper {
        CLOSED(.545),
        PARTIAL(.6275),
        OPEN(.71);

        public final double position;
        Dropper(double position) {this.position = position;}
    }

    public RobotConfig(HardwareMap hardwareMap) {
        //Drivetrain
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftRear");
        backRight = hardwareMap.get(DcMotorEx.class, "rightRear");

        //Remaining motors
        climbMotor = hardwareMap.get(DcMotorEx.class, "climbMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        climbMotor.setTargetPosition(0);
        climbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climbMotor.setPower(1);

        if (!PassData.slidesInitiated) {
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            PassData.slidesInitiated = true;
        }

        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveTrainMotors = Arrays.asList(frontLeft, frontRight, backLeft, backRight);
        allMotors = Arrays.asList(frontLeft, frontRight, backLeft, backRight, climbMotor, intakeMotor, transferMotor, slideMotor);

        //Servos
        dropServo = hardwareMap.servo.get("dropServo"); // ex hub 2
        leftPixelServo = hardwareMap.servo.get("leftServo"); // control hub 0
        rightPixelServo = hardwareMap.servo.get("rightServo"); // ex hub 0
        droneServo = hardwareMap.servo.get("droneServo"); // ex hub 1
        leftStackServo = hardwareMap.servo.get("leftSpikeMarkServo"); // control hub 2
        rightStackServo = hardwareMap.servo.get("rightSpikeMarkServo"); // ex hub 4
        stackHoldServo = hardwareMap.servo.get("rightLiftServo");

        rightStackServo.setDirection(Servo.Direction.REVERSE);

        //Slide encoder
        slideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "slideMotor"));


        //Using manual bulk reads for slightly better performance
        //Must clear cache to get new results
        allHubs = hardwareMap.getAll(LynxModule.class);

        manualBulkReads(true);

    }

    public void manualBulkReads(boolean manualReads) {
        if (manualReads) {
            for (LynxModule hub : allHubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
        } else {
            for (LynxModule hub : allHubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
        }
    }

    public void clearBulkCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    public void update() {
        odometry.update();
        updateLift(timer.seconds());

        //Must be called to get new results
        clearBulkCache();
    }

    public Pose2d getPoseEstimate() {
        return odometry.getPose();
    }

    //Intake code
    public void stackArm(StackArm stackArm) {
        leftStackServo.setPosition(stackArm.position);
        rightStackServo.setPosition(stackArm.position);
    }

    //Outtake code
    public void dropper(Dropper dropper) {
        dropServo.setPosition(dropper.position);
    }

    //Slide code
    public void setTargetSlidePos(double targetPos) {
        targetSlidePos = Range.clip(targetPos, 0, 35);
    }

    public void setTargetSlidePos(SlideHeight height) {
        targetSlidePos = height.height;
    }

    public void rawSetTargetSlidePos(double targetPos) {
        targetSlidePos = targetPos;
    }

    public double getTargetSlidePos() {
        return targetSlidePos;
    }

    public double slidePosInches() {
        return (double) slideEncoder.getCurrentPosition() * slideToInches;//1.6 diameter * pi / 8192
    }

    public void updateLift(double currentTime) {
        //Error is positive when slide have to up
        double error = targetSlidePos - slidePosInches();
        double absError = Math.abs(error);

        double p, d;

        //Checks if error is in acceptable amounts
        if (absError < .2) {
            d = 0;
            //If slides are at bottom give 0 power, otherwise slight power bc gravity
            if (slidePosInches() < .2) {
                p = 0;
            } else {
                p = .1;
            }
        } else {
            p = error;
            d = (error-prevError)/(currentTime-prevTime);
            //Add ifs to slow down more as it approaches
        }

        slideMotor.setPower(p+d);
        prevError = error;
        prevTime = currentTime;

    }


}
