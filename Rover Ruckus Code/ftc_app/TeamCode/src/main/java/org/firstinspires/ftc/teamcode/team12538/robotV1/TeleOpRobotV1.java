package org.firstinspires.ftc.teamcode.team12538.robotV1;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.team12538.components.MineralMechanism;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

public class TeleOpRobotV1 extends RobotBase {
    private boolean isLatched = false;

    @Override
    public void init() {
        super.init();
    }

    public void player1controls(Gamepad gamepad) {
        driveRobot(gamepad);

        // Intake controls
        // Move to Kevin's control
        /*
        if(gamepad.right_bumper) {
            collector.enableIntake(MineralMechanism.Direction.InTake);
        } else if(gamepad.left_bumper) {
            collector.enableIntake(MineralMechanism.Direction.OutTake);
        } else {
            collector.disableIntake();
        }
        */

        // latch controls
        if(gamepad.x) {
            robotLatch.latchClose();
        } else if(gamepad.b) {
            robotLatch.latchOpen();
        }

        // scissor lift controls
        if(gamepad.y) {
            robotLatch.powerLiftOnUpPosition(1.0, 2000);
        } else if(robotLatch.isScissorLiftBusy()) {
            if (gamepad.dpad_up) {
                // make sure intake flip is in the up position
                collector.flipCollectorBox(collector.intakeFlipUpPos);
                robotLatch.powerLift(1.0);
            } else if (gamepad.dpad_down) {
                robotLatch.powerLift(-1.0);
            } else {
                robotLatch.powerLift(0d);
            }
        }
    }

    public void player2Controls(Gamepad gamepad) {
        // intake box controls
        if(gamepad.y) {
            // toggle collector flip/unflip
            collector.flipCollectorBox(collector.intakeFlipUpPos);
        } else if(gamepad.a) {
            collector.flipCollectorBox(collector.intakeFlipDownPos);
        } else if(gamepad.x) {
            // lift collector to prepare position
            collector.flipCollectorBox(collector.intakeFlipPrepPos);
        } else if(gamepad.b) {
            // auto deposit minerals to outtake box
            collector.autoMineralDeposit();
        } else {
            // arm extension control
            collector.controlArmExt(-gamepad.left_stick_x);
        }

        // Intake controls
        if(gamepad.right_bumper) {
            collector.enableIntake(MineralMechanism.Direction.InTake);
        } else if(gamepad.left_bumper) {
            collector.enableIntake(MineralMechanism.Direction.OutTake);
        } else {
            collector.disableIntake();
        }

        // deposit box controls
        if(gamepad.dpad_up) {
            collector.liftDepo(3350);
        } else if(gamepad.dpad_down) {
            collector.lowerDepo();
        } else if(gamepad.right_trigger > 0 && collector.canFlipDepoBox()) {
            collector.rotateDepositBox(1d);
        }
    }
}
