package org.firstinspires.ftc.teamcode.team12538.robotV1;

import com.qualcomm.robotcore.hardware.Gamepad;

public class TeleOpRobotV1 extends RobotBase {
    private boolean isLatched = false;
    private double leg_position = 0.0;

    private double prevCollectorBoxPosition;

    @Override
    public void init() {
        super.init();
        prevCollectorBoxPosition = getCollector().getLeftFlip().getPosition();
    }

    public void player1controls(Gamepad gamepad) {
        // Drive mode for mecanum wheel
        double r = Math.hypot(gamepad.left_stick_x, gamepad.left_stick_y);
        double robotAngle = Math.atan2(gamepad.left_stick_y, gamepad.left_stick_x) - Math.PI / 4;
        double rightX = gamepad.right_stick_x;

        final double v1 = r * Math.sin(robotAngle) - rightX;
        final double v2 = r * Math.cos(robotAngle) + rightX;
        final double v3 = r * Math.cos(robotAngle) - rightX;
        final double v4 = r * Math.sin(robotAngle) + rightX;

        double power = 1.0;
        if(gamepad.left_trigger > 0 || gamepad.right_trigger > 0) {
            power = 0.3; // slowdown robot on left or right trigger
        }

        frontLeftDrive.setPower(power * Math.signum(v1));
        frontRightDrive.setPower(power * Math.signum(v2));
        rearLeftDrive.setPower(power * Math.signum(v3));
        rearRightDrive.setPower(power * Math.signum(v4));

        if(gamepad.x) {
            robotLatch.teleHook();
        } else if(gamepad.a) {
            robotLatch.teleUnhook();
        }

        if (gamepad.dpad_up) {
            robotLatch.powerLift(1.0);
            if(robotLatch.shouldLowerSupportLeg()) {
                robotLatch.autoLegDown();
            } else {
                robotLatch.autoLegUp();
            }
        } else if (gamepad.dpad_down) {
            robotLatch.powerLift(-1.0, 100);
            if(robotLatch.shouldLowerSupportLeg()) {
                robotLatch.autoLegDown();
            } else {
                robotLatch.autoLegUp();
            }
        } else if(gamepad.y) {
            robotLatch.powerLift(-1.0);
            if(robotLatch.shouldLowerSupportLeg()) {
                robotLatch.autoLegDown();
            } else {
                robotLatch.autoLegUp();
            }
        } else {
            robotLatch.powerLift(0d);
        }
    }

    public void player2Controls(Gamepad gamepad) {
        if(gamepad.x) {
            // lower
            collector.flipCollectorBox(0.9);
            // collector.enableIntake(MineralMechanism.Direction.InTake, true);
        } else if(gamepad.a) {
            // prepare
            collector.flipCollectorBox(0.6);
        } else if (gamepad.b){
            // deposit
            collector.flipCollectorBox(0.2);
        }

        if(gamepad.y) {
            // auto deposit minerals to outtake box
            collector.autoMineralDeposit();
        } else {
            // arm extension control
            collector.controlArmExt(-gamepad.left_stick_x);
        }

        // deposit box controls
        if(gamepad.dpad_up) {
            collector.liftDepo();
        } else if(gamepad.dpad_down) {
            collector.lowerDepo();
        } else if(gamepad.right_bumper) {
            collector.getDepo().setPosition(0.8);
        } else {
            collector.getDepo().setPosition(0d);
        }
    }
}
