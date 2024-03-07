package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robot.PassData;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;


import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;


import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;
import static java.lang.Double.isNaN;
import static java.lang.Math.abs;
import static java.lang.Math.max;

import android.annotation.SuppressLint;

@Config
public class RobotConfig extends MecanumDrive {

    //Roadrunner stuff
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(21, .5, 1.4);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(19, .5, 1.4);

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;

    private VoltageSensor batteryVoltageSensor;

    private List<Integer> lastEncPositions = new ArrayList<>();
    private List<Integer> lastEncVels = new ArrayList<>();

    public TwoWheelTrackingLocalizer localizer;
    public BackDropLocalizer backDropLocalizer;

    List<LynxModule> allHubs;

    ElapsedTime stackTimer = new ElapsedTime();
    ElapsedTime slideTimer = new ElapsedTime();

    //Hardware Components
    public DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public List<DcMotorEx> driveTrainMotors;

    public DcMotorEx slideMotor, climbMotor, intakeMotor, transferMotor;
    public List<DcMotorEx> allMotors;

    public double targetSlidePos = 0;//, prevError = 0, prevTime = 0;

    public double prevError = 0;

    public Encoder slideEncoder;
    public final double slideToInches = 1.6 * Math.PI / 8192;

    public Servo dropServo, leftPixelServo, rightPixelServo, droneServo, leftStackServo, rightStackServo, stackHoldServo;

    public StandardTrackingWheelLocalizer threeWheelLocalizer;
    public IMU controlIMU;
    public BNO055IMUNew expansionIMU;

    public OpenCvCamera webcam, webcamR;

    //Pathing stuff
    boolean followPurePursuitPath = false;
    boolean followRoadrunnerPath = false;
    Path currentPath;

    public enum CurrentIMU {
        expansionIMU,
        controlIMU,
        threeWheel,
        noIMU;
    }
    CurrentIMU currentIMU = CurrentIMU.expansionIMU;

    public double externalHeading = 0, externalHeadingVel = 0;

    public enum StackArm {
        GUIDE(1, 1 + RobotConstants.rightSpikeOffset),
        FAR_OUT(0.52, .52 + RobotConstants.rightSpikeOffset),
        OUT(.59, .59 + RobotConstants.rightSpikeOffset),
        IN(.74,  .74 + RobotConstants.rightSpikeOffset),
        FAR_LEFT(.2, .59 + RobotConstants.rightSpikeOffset),
        FAR_RIGHT(.59, .2 + RobotConstants.rightSpikeOffset);

        public final double position, position2;
        StackArm(double position, double position2) {
            this.position = position;
            this.position2  = position2;
        }
    }

    public enum AutoStackCycle {
        HOLD_2,
        GRAB_2,
        RELEASE_2,
        HOLD_1,
        GRAB_1,
        RELEASE_1,
        RELEASE_HOLD,
        WAIT
    }
    AutoStackCycle autoStackCycle = AutoStackCycle.WAIT;

    public enum SlideHeight {
        BOTTOM(-.2),
        PRELOAD_DROP(11),
        LOW(15),
        MEDIUM(21),
        HIGH(30);

        public final double height;
        SlideHeight(double height) {this.height = height;}
    }

    public enum Dropper {
        CLOSED(.545),
        PARTIAL(.6275),
        OPEN(.71);

        public final double position;
        Dropper(double position) {this.position = position;}
    }

    public RobotConfig(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        //Using manual bulk reads for slightly better performance
        //Must clear cache to get new results
        allHubs = hardwareMap.getAll(LynxModule.class);

        manualBulkReads(true);

        //Drivetrain
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftRear");
        backRight = hardwareMap.get(DcMotorEx.class, "rightRear");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        driveTrainMotors = Arrays.asList(frontLeft, backLeft, backRight, frontRight);

        for (DcMotorEx motor : driveTrainMotors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        threeWheelLocalizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);

        controlIMU = hardwareMap.get(BHI260IMU.class, "imu");
        expansionIMU = hardwareMap.get(BNO055IMUNew.class, "imuExpansion");

//        Orientation expansionHubOrientation = RevHubOrientationOnRobot.xyzOrientation(90, 90, -37.533568);
//        RevHubOrientationOnRobot expansionOrientationOnRobot = new RevHubOrientationOnRobot(expansionHubOrientation);

        BNO055IMUNew.Parameters parameters = new BNO055IMUNew.Parameters(
                new RevHubOrientationOnRobot(
                new Orientation(AxesReference.INTRINSIC,
                AxesOrder.XYZ, AngleUnit.DEGREES,
                        90, 90, 52.466332f, 0))//52.466332f
        );//-90, 90, 52.466332f

        expansionIMU.initialize(parameters);
        expansionIMU.resetYaw();

        Orientation hubOrientation = RevHubOrientationOnRobot.xyzOrientation(90, -90, -52.466332f);
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(hubOrientation);

        controlIMU.initialize(new IMU.Parameters(orientationOnRobot));

        controlIMU.resetYaw();

        localizer = new TwoWheelTrackingLocalizer(hardwareMap, this);

        localizer.setPoseEstimate(PassData.currentPose);

        setLocalizer(localizer);

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, HEADING_PID, batteryVoltageSensor,
                lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
        );

        backDropLocalizer = new BackDropLocalizer(hardwareMap);

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

        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        //Cameras
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"));
        webcamR = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "cameraR"));


        //Slide encoder
        slideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "slideMotor"));

        clearBulkCache();
        if (PassData.slidesInitiated && slidePosInches()>-.1) {
            //If slides are not at the bottom they are set to their current pose
            if (slidePosInches()>1) {
                targetSlidePos = slidePosInches();
            }
        } else {
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            PassData.slidesInitiated = true;
        }


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
        updateIMU();

        updatePoseEstimate();

        updateLift();
        updateIntake();

        if (true) {//followRoadrunnerPath
            DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
            if (signal != null) setDriveSignal(signal);
            if (!isBusy()) {
                followRoadrunnerPath = false;
            }
        }

        if (followPurePursuitPath) {
            //localizer.getPoseEstimate().getX(), localizer.getPoseEstimate().getY()
            double[] driveTrainPowers = currentPath.loop(localizer.getPoseEstimate().getX(), localizer.getPoseEstimate().getY(), localizer.getPoseEstimate().getHeading()+Math.PI/2);

            driveRobotCentric(driveTrainPowers[0], driveTrainPowers[1], driveTrainPowers[2]);
//            setMecanumDrive(-driveTrainPowers[1], -driveTrainPowers[0], -driveTrainPowers[2]);

            //stops the path from being used once it is done
            if (currentPath.isFinished()) {
                followPurePursuitPath = false;
                setMotorPowers(0, 0 ,0, 0);
            }
        }

        //Must be called to get new results
        clearBulkCache();
    }

    @SuppressLint("DefaultLocale")
    public String motorPowers() {
        return String.format("%.2f, %.2f, %.2f, %.2f", frontLeft.getPower(), backLeft.getPower(), backRight.getPower(), frontRight.getPower());
    }

    public void exit() {
        PassData.currentPose = localizer.getPoseEstimate();

    }

    public void releaseDrone() {
        droneServo.setPosition(RobotConstants.droneRelease);
    }

    //Intake code
    public void stackArm(StackArm stackArm) {
        leftStackServo.setPosition(stackArm.position);
        rightStackServo.setPosition(stackArm.position2);
    }

    public void stackHold(boolean holdStack) {
        if (holdStack) {
            stackHoldServo.setPosition(.6);
        } else {
            stackHoldServo.setPosition(.1);
        }
    }

    public void grabFromStack(double pixels) {
        stackTimer.reset();
        stackHold(true);

        if (pixels == 1) {
            autoStackCycle = AutoStackCycle.HOLD_1;
        } else if (pixels == 2) {
            autoStackCycle = AutoStackCycle.HOLD_2;
        }
    }

    public void updateIntake() {
        switch (autoStackCycle) {
            case HOLD_2:
                if (stackTimer.seconds()>.2) {
                    stackArm(StackArm.IN);
                    autoStackCycle = AutoStackCycle.GRAB_2;
                }
                break;
            case GRAB_2:
                if (stackTimer.seconds()>.4) {
                    stackArm(StackArm.OUT);
                    autoStackCycle = AutoStackCycle.RELEASE_2;
                }
                break;
            case RELEASE_2:
                if (stackTimer.seconds()>.6) {
                    stackArm(StackArm.IN);
                    stackTimer.reset();
                    autoStackCycle = AutoStackCycle.GRAB_1;
                }
                break;
            case HOLD_1:
                if (stackTimer.seconds()>.2) {
                    stackArm(StackArm.IN);
                    stackTimer.reset();
                    autoStackCycle = AutoStackCycle.GRAB_1;
                }
                break;
            case GRAB_1:
                if (stackTimer.seconds()>.4) {
                    stackArm(StackArm.OUT);
                    autoStackCycle = AutoStackCycle.RELEASE_1;
                }
                break;
            case RELEASE_1:
                if (stackTimer.seconds()>.5) {
                    stackHold(false);
                    stackTimer.reset();
                    autoStackCycle = AutoStackCycle.WAIT;
                }
                break;
//            case RELEASE_HOLD:
//                if (stackTimer.seconds()>.5) {
//
//                }

        }
    }

    //Outtake code
    public void dropper(Dropper dropper) {
        dropServo.setPosition(dropper.position);
    }

    //Slide code
    public void setTargetSlidePos(double targetPos) {
        targetSlidePos = Range.clip(targetPos, 0, 35);//35
    }

    public void ResetSlides() {
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setTargetSlidePos(SlideHeight.BOTTOM);
        PassData.slidesInitiated = true;
    }

    public void setTargetSlidePos(SlideHeight height) {
        targetSlidePos = height.height;//Range.clip(height.height, -3, 26)
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

    public void updateLift() {//double currentTime
        //Error is positive when slide have to up
        double error = targetSlidePos - slidePosInches();

        double p, d = 0;

        //Checks if error is in acceptable amounts
        if (error<.1 && error>-.1) {
//            d = 0;
            //If slides are at bottom give 0 power, otherwise slight power bc gravity
            if (slidePosInches() < .1) {
                p = 0;
            } else {
                p = .04;//*Math.signum(error)
            }
        } else if (error>4 || error<-7) {
            //Slides set to max power
            p = Math.signum(error);
//            d = 0;
        } else if (error>0){
            //with in 4 in but has to move up
            p = error*.3;
            d = ((error - prevError) / slideTimer.seconds()) * .002;
        } else if (error<-4){
            //with in 6 and greater than 4 but has to move down
            p = error*.2;
            d = ((error-prevError)/slideTimer.seconds())*.005;
        } else {//if (error<-2)
            //with in 4 in but has to move down
            p = error*.1-.15;
            if (slidePosInches()>.2) {
                d = ((error - prevError) / slideTimer.seconds()) * .007;//.007
            }
        }

        slideMotor.setPower(p + d);
        slideTimer.reset();
        prevError = error;
//        prevError = error;
//        prevTime = currentTime;

    }

    //Drivetrain functions
    public void followPurePursuitPath(Path path) {
        path.init();
        currentPath = path;
        followPurePursuitPath = true;
    }

    public boolean isPurePursuitPathFinished() {
        return currentPath.isFinished();
    }


    public void setMecanumDrive(double forward, double strafe, double turn) {
        //Find value to make all motor powers less than 1
        double scalePower = max(abs(forward) + abs(strafe) + abs(turn), 1);

        //Creating string with all drive powers for mecanum drive
        Double[] driveSpeeds = {(forward-strafe-turn)/scalePower, (forward+strafe-turn)/scalePower,
                (forward-strafe+turn/scalePower), (forward+strafe+turn)/scalePower};

        //Setting motors to their new powers
        setMotorPowers(driveSpeeds[0], driveSpeeds[1], driveSpeeds[2], driveSpeeds[3]);
    }

    public void setMecanumDriveFieldCentric(double forward, double strafe, double turn, double heading) {

        //rotating Joystick values to account for robot heading
        double rotatedForward = forward*Math.cos(-heading)-strafe*Math.sin(-heading),
                rotatedStrafe = forward*Math.sin(-heading)+strafe*Math.cos(-heading);

        //Find value to make all motor powers less than 1
        double scalePower = max(abs(rotatedForward) + abs(rotatedStrafe) + abs(turn), 1);

        //Creating string with all drive powers for mecanum drive
        Double[] driveSpeeds = {(rotatedForward-rotatedStrafe-turn)/scalePower, (rotatedForward+rotatedStrafe-turn)/scalePower,
                (rotatedForward-rotatedStrafe+turn/scalePower), (rotatedForward+rotatedStrafe+turn)/scalePower};

        //Setting motors to their new powers
        setMotorPowers(driveSpeeds[0], driveSpeeds[1], driveSpeeds[2], driveSpeeds[3]);
    }

    public void setMecanumDriveHeadingPriority(double forward, double strafe, double turn) {
        turn = Range.clip(turn, -.8, .8);

        double remainingPower = 1-abs(turn);
        //Find value to make all motor powers less than 1
        double scalePower = max((abs(forward) + abs(strafe))/remainingPower, 1/remainingPower);

        //Creating string with all drive powers for mecanum drive
        Double[] driveSpeeds = {(forward-strafe)/scalePower-turn, (forward+strafe)/scalePower-turn,
                (forward-strafe)/scalePower+turn, (forward+strafe)/scalePower+turn};

        //Setting motors to there new powers
        setMotorPowers(driveSpeeds[0], driveSpeeds[1], driveSpeeds[2], driveSpeeds[3]);
    }

    public void setMecanumDriveFieldCentricHeadingPriority(double forward, double strafe, double turn, double heading) {
        turn = Range.clip(turn, -.8, .8);

        //rotating Joystick values to account for robot heading
        double rotatedForward = forward*Math.cos(-heading)-strafe*Math.sin(-heading),
                rotatedStrafe = forward*Math.sin(-heading)+strafe*Math.cos(-heading);

        double remainingPower = 1-abs(turn);
        //Find value to make all motor powers less than 1
        double scalePower = max((abs(rotatedForward) + abs(rotatedStrafe))/remainingPower, 1/remainingPower);

        //Creating string with all drive powers for mecanum drive
        Double[] driveSpeeds = {(rotatedForward-rotatedStrafe)/scalePower-turn, (rotatedForward+rotatedStrafe)/scalePower-turn,
                (rotatedForward-rotatedStrafe)/scalePower+turn, (rotatedForward+rotatedStrafe)/scalePower+turn};

        //Setting motors to their new powers
        setMotorPowers(driveSpeeds[0], driveSpeeds[1], driveSpeeds[2], driveSpeeds[3]);
    }

    //localizer stuff
    public void updateBackdropLocalizer() {
        backDropLocalizer.update();
    }

    public Pose2d getBackdropPoseEstimate() {
        return backDropLocalizer.getPoseEstimate(localizer.getPoseEstimate());
    }

    public void relocalizeBackdrop() {
        updateBackdropLocalizer();
        localizer.setPoseEstimate(getBackdropPoseEstimate());
    }

    public boolean safeRelocalizeBackdrop() {
//        updateBackdropLocalizer();
//        if (backDropLocalizer.isInRange(getPoseEstimate(),.5, Math.toRadians(2))) {
//            localizer.setPoseEstimate(getBackdropPoseEstimate());
//            return true;
//        } else {
//            return false;
//        }
        return false;
    }

    //Roadrunner functions
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        followRoadrunnerPath = true;
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        followRoadrunnerPath = true;
        turnAsync(angle);
        waitForIdle();
    }

    public void cancelTrajectory() {
        trajectorySequenceRunner.cancelTrajectory();
    }

    //Add code for cancel and pause later
    public void pauseTrajectory() {
        cancelTrajectory();
    }

    public void resumeTrajectory() {

    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        followRoadrunnerPath = true;
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followRoadrunnerPath = true;
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        followRoadrunnerPath = true;
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followRoadrunnerPath = true;
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

//    public void update() {
//        updatePoseEstimate();
//        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
//        if (signal != null) setDriveSignal(signal);
//    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : driveTrainMotors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : driveTrainMotors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : driveTrainMotors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        lastEncPositions.clear();

        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : driveTrainMotors) {
            int position = motor.getCurrentPosition();
            lastEncPositions.add(position);
            wheelPositions.add(encoderTicksToInches(position));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        lastEncVels.clear();

        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : driveTrainMotors) {
            int vel = (int) motor.getVelocity();
            lastEncVels.add(vel);
            wheelVelocities.add(encoderTicksToInches(vel));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double fL, double bL, double bR, double fR) {
        frontLeft.setPower(fL);
        backLeft.setPower(bL);
        backRight.setPower(bR);
        frontRight.setPower(fR);
    }

    //
    public void checkSensors() {
            if (!isNaN(controlIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS))) {
                currentIMU = CurrentIMU.controlIMU;
                resumeTrajectory();
            } else if (!isNaN(expansionIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS))) {
                currentIMU = CurrentIMU.expansionIMU;
                resumeTrajectory();
            }
    }

    public CurrentIMU getCurrentIMU() {
        return currentIMU;
    }

    public void setCurrentIMU(CurrentIMU imu) {currentIMU = imu;}

    public void updateIMU() {
        switch (currentIMU) {
            case controlIMU:
                externalHeading = controlIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                if (isNaN(externalHeading)) {
                    externalHeading = expansionIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    if (isNaN(externalHeading)) {
                        currentIMU = CurrentIMU.noIMU;
                        threeWheelLocalizer.update();
                        threeWheelLocalizer.setPoseEstimate(localizer.getPoseEstimate());
                        setLocalizer(threeWheelLocalizer);
                    } else {
                        currentIMU = CurrentIMU.expansionIMU;
                    }
                } else {
                    externalHeadingVel = controlIMU.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
                }
                break;
            case expansionIMU:
                externalHeading = expansionIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                if (isNaN(externalHeading)) {
                    externalHeading = controlIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    if (isNaN(externalHeading)) {
                        currentIMU = CurrentIMU.noIMU;
                        threeWheelLocalizer.update();
                        threeWheelLocalizer.setPoseEstimate(localizer.getPoseEstimate());
                        setLocalizer(threeWheelLocalizer);
                    } else {
                        currentIMU = CurrentIMU.controlIMU;
                    }
                } else {
                    externalHeadingVel = expansionIMU.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
                }
                break;
            case noIMU:
//                checkSensors();
        }

    }

    @Override
    public double getRawExternalHeading() {
        return externalHeading;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return  externalHeadingVel;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    //Pure Pursuit methods
    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, 0.0);
    }

    protected void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if (maxMagnitude > 1) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude);
            }
        }

    }

    protected void normalize(double[] wheelSpeeds, double magnitude) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        for (int i = 0; i < wheelSpeeds.length; i++) {
            wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude) * magnitude;
        }

    }

    public void driveFieldCentric(double strafeSpeed, double forwardSpeed,
                                  double turnSpeed, double gyroAngle) {
        strafeSpeed = Range.clip(strafeSpeed, -1, 1);
        forwardSpeed = Range.clip(forwardSpeed, -1, 1);
        turnSpeed = Range.clip(turnSpeed, -1, 1);

        Vector2d input = new Vector2d(strafeSpeed, forwardSpeed);
        input = input.rotateBy(-gyroAngle);

        double theta = input.angle();

        double[] wheelSpeeds = new double[4];
        wheelSpeeds[RobotDrive.MotorType.kFrontLeft.value] = Math.sin(theta + Math.PI / 4);
        wheelSpeeds[RobotDrive.MotorType.kFrontRight.value] = Math.sin(theta - Math.PI / 4);
        wheelSpeeds[RobotDrive.MotorType.kBackLeft.value] = Math.sin(theta - Math.PI / 4);
        wheelSpeeds[RobotDrive.MotorType.kBackRight.value] = Math.sin(theta + Math.PI / 4);

        normalize(wheelSpeeds, input.magnitude());

        wheelSpeeds[RobotDrive.MotorType.kFrontLeft.value] += turnSpeed;
        wheelSpeeds[RobotDrive.MotorType.kFrontRight.value] -= turnSpeed;
        wheelSpeeds[RobotDrive.MotorType.kBackLeft.value] += turnSpeed;
        wheelSpeeds[RobotDrive.MotorType.kBackRight.value] -= turnSpeed;

        normalize(wheelSpeeds);

        driveWithMotorPowers(
                wheelSpeeds[RobotDrive.MotorType.kFrontLeft.value],
                wheelSpeeds[RobotDrive.MotorType.kFrontRight.value],
                wheelSpeeds[RobotDrive.MotorType.kBackLeft.value],
                wheelSpeeds[RobotDrive.MotorType.kBackRight.value]
        );
    }

    public void driveWithMotorPowers(double frontLeftSpeed, double frontRightSpeed,
                                     double backLeftSpeed, double backRightSpeed) {
        setMotorPowers(frontLeftSpeed, backLeftSpeed, backRightSpeed, frontRightSpeed);
    }
}
