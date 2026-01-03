package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class RumbleTest extends OpMode {
    boolean wasA, isA;
    double endGameStart;
    boolean isEndGame;
    @Override
    public void init(){

    }
    @Override
    public void start() {
        endGameStart = getRuntime() + 90;

    }

    @Override
    public void loop() {
        isA = gamepad1.a;
//        if (gamepad1.a) {
//            gamepad1.rumble(100);
        if (endGameStart >= getRuntime() && !isEndGame) {
            gamepad1.rumbleBlips(3);
            isEndGame = true;
        }
//
        if (isA && !wasA) {
            gamepad1.rumbleBlips(3);
            gamepad1.rumble(1.0, 0, 100); //left joystick 1 rumble right joystick no rumble
        }
        wasA = isA;
    }
}
