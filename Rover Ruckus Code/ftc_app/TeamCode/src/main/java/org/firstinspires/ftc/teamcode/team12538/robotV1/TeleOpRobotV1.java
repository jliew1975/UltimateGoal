package org.firstinspires.ftc.teamcode.team12538.robotV1;

import com.qualcomm.robotcore.hardware.DcMotor;
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

        if (gamepad.right_bumper || gamepad.dpad_down) {
            robotLatch.powerLift(-1.0);
        } else if(gamepad.left_bumper || gamepad.dpad_up) {
            collector.getIntakeFlip().setPosition(collector.intakeFlipPrepPos);
            robotLatch.powerLift(1.0);
        } else {
            robotLatch.powerLift(0d);
        }
    }

    public void player2Controls(Gamepad gamepad) {

        // intake box controls
        if (gamepad.y) {
            // toggle collector flip/unflip
            collector.flipCollectorBox(collector.intakeFlipUpPos);
        } else if (gamepad.a) {
            collector.flipCollectorBox(collector.intakeFlipDownPos);
        } else if (gamepad.x) {
            // lift collector to prepare position
            collector.flipCollectorBox(collector.intakeFlipPrepPos);
        } else if (gamepad.b) {
            // auto deposit minerals to outtake box
            collector.autoMineralDeposit();
        } else {
            // arm extension control
            collector.controlArmExt(-gamepad.left_stick_x);
        }

        // Intake controls
        if (gamepad.right_bumper) {
            collector.enableIntake(MineralMechanism.Direction.InTake);
        } else if (gamepad.left_bumper) {
            collector.enableIntake(MineralMechanism.Direction.OutTake);
        } else {
            collector.disableIntake();
        }

        // deposit box controls
        if (gamepad.dpad_up) {
            collector.liftDepo(700);
        } else if (gamepad.dpad_down) {
            collector.lowerDepo();
        }

        // depo flip
        if(gamepad.right_trigger > 0.1) {
            collector.liftDepo(700, false, true);
        } else if(collector.canFlipDepoBox() && gamepad.left_trigger > 0.1) {
            collector.rotateDepositBox(collector.depoFlipPos);
        }
    }
}
