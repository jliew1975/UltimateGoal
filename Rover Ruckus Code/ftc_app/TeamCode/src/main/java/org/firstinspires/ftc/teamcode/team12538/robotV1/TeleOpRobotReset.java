package org.firstinspires.ftc.teamcode.team12538.robotV1;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.team12538.components.MineralMechanism;
import org.firstinspires.ftc.teamcode.team12538.components.MineralMechanism.MineralSide;

public class TeleOpRobotReset extends TeleOpRobotV1 {
    private boolean isLatched = false;

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void player1controls(Gamepad gamepad) {
        if (gamepad.dpad_up) {
            robotLatch.powerLift(1.0);
        } else if (gamepad.dpad_down) {
            robotLatch.powerLift(-1.0);
        } else {
            robotLatch.powerLift(0d);
        }

        if(gamepad.x) {
            // lower
            collector.flipCollectorBox(0d);
        } else if(gamepad.a) {
            // prepare
            collector.flipCollectorBox(0.6);
        } else if (gamepad.b){
            // deposit
            collector.flipCollectorBox(1d);
        }

        // arm extension control
        collector.controlArm(-gamepad.left_stick_x);
    }

    @Override
    public void player2Controls(Gamepad gamepad) {
        if (gamepad.dpad_up) {
            robotLatch.powerLift(1.0);
        } else if (gamepad.dpad_down) {
            robotLatch.powerLift(-1.0);
        } else {
            robotLatch.powerLift(0d);
        }

        if(gamepad.x) {
            // lower
            collector.flipCollectorBox(0d);
        } else if(gamepad.a) {
            // prepare
            collector.flipCollectorBox(0.6);
        } else if (gamepad.b){
            // deposit
            collector.flipCollectorBox(1d);
        }

        // arm extension control
        collector.controlArm(-gamepad.left_stick_x);
    }
}
