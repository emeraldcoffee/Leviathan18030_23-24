//end game alert sends three rumbles to the driver to alert them about endgame
//resource: LearnJavaForFTC page 107
package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class endGameAlert extends OpMode{
    boolean inEndGame;
    double endGameStart;

    @Override
    public void init() {
        inEndGame = false;
    }

    @Override
    public void start() {
        //amount of time in teleop before endgame
        endGameStart = getRuntime() + 90;
    }
    @Override
    public void loop() {
        if ((getRuntime()> endGameStart) && !inEndGame){
            //rumbles for both gamepads
            gamepad2.rumbleBlips(3);
            gamepad1.rumbleBlips(3);
            inEndGame = true;
        }
    }
}
