package org.firstinspires.ftc.teamcode.robot.oldCode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp
public class testTeleOp extends LinearOpMode {
    //makes sure that the servo has enough time to open
    ElapsedTime dropTimer = new ElapsedTime();
    int liftPosition;

    enum LiftState {

        LIFT_START,
        LIFT_EXTEND,
        LIFT_OUTTAKE,
        LIFT_OUTTAKE_DROPPING,
        LIFT_OUTTAKE_RELOADING,
        LIFT_RETRACT
    }


    @Override
    public void runOpMode() throws InterruptedException {
        //makes a variable, referencing LiftState called liftState and setting it to LIFT_START
        LiftState liftState = LiftState.LIFT_START;
        waitForStart();
        while (opModeIsActive()) {
            switch (liftState) {
                case LIFT_START:
                    if (gamepad2.x) {
                        //set slides to extend
                        liftState = LiftState.LIFT_EXTEND;
                        //test value
                        liftPosition = 500;
                    } else if (gamepad2.a){
                        //set slides to extend
                        liftState = LiftState.LIFT_EXTEND;
                        //test value
                        liftPosition = 250;
                    } else if (gamepad2.b) {
                        //set slides to extend
                        liftState = LiftState.LIFT_EXTEND;
                        //test value
                        liftPosition = 750;
                    }
                break;
                //Checks if slides are in the right position
                case LIFT_EXTEND:
                    if(true/*position*/)
                    {
                        //goes to next state
                        //drops the hexagons
                        liftState = LiftState.LIFT_OUTTAKE;
                    }
                break;
                case LIFT_OUTTAKE:
                    //add parameter to make sure you don't set the position to your current position
                    if(gamepad1.left_bumper) {
                        //set servo to a certain position
                        //resets drop timer
                        dropTimer.reset();
                        //sets state to dropping
                        liftState = LiftState.LIFT_OUTTAKE_DROPPING;
                    //this series of if statements give the driver a second chance to set the lift position in case of mis-clicking
                    } else if (gamepad2.y) {
                        //moves slides back to the bottom
                        liftState = LiftState.LIFT_RETRACT;
                        //test value
                        liftPosition = 0;
                    } else if (gamepad2.x) {
                        //set slides to extend
                        liftState = LiftState.LIFT_EXTEND;
                        //test value
                        liftPosition = 500;
                    } else if (gamepad2.a){
                        //set slides to extend
                        liftState = LiftState.LIFT_EXTEND;
                        //test value
                        liftPosition = 250;
                    } else if (gamepad2.b) {
                        //set slides to extend
                        liftState = LiftState.LIFT_EXTEND;
                        //test value
                        liftPosition = 750;
                    }
                    break;
                case LIFT_OUTTAKE_DROPPING:
                    if(dropTimer.seconds() > 0.3){
                        //set servo to closing
                        //lift state to closing
                        dropTimer.reset();
                        liftState = LiftState.LIFT_OUTTAKE_RELOADING;
                    }
                break;
                case LIFT_OUTTAKE_RELOADING:
                    if(dropTimer.seconds() > 0.3) {
                        //set servo to closing
                        //lift state to closing
                        liftState = LiftState.LIFT_OUTTAKE;
                    }
                break;
                case LIFT_RETRACT:
                    if(true/*position*/)
                    {
                        //goes to next state
                        //drops the hexagons
                        liftState = LiftState.LIFT_START;
                    }
                break;
            }
        }
    }
}
