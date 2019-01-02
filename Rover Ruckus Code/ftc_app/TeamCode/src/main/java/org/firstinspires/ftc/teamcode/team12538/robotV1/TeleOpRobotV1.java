package org.firstinspires.ftc.teamcode.team12538.robotV1;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.team12538.components.EventCallback;
import org.firstinspires.ftc.teamcode.team12538.components.MineralMechanism;

import static org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils.sleep;

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

        // Intake controls
        if(gamepad.right_bumper) {
            if(collector.getCollectorBoxPosition() == 0.9) {
                collector.flipCollectorBox(1d);
            }
            collector.enableIntake(MineralMechanism.Direction.InTake);
        } else if(gamepad.left_bumper) {
            collector.enableIntake(MineralMechanism.Direction.OutTake);
        } else {
            if(collector.getCollectorBoxPosition() == 1d) {
                collector.flipCollectorBox(0.9);
            }
            collector.disableIntake();
        }

        // latch controls
        if(gamepad.x) {
            collector.flipCollectorBox(0.6);
            collector.liftDepo(2000, false, new EventCallback() {
                @Override
                public void callbackEvent() {
                    robotLatch.teleHook();
                    collector.flipCollectorBox(0.19);
                }
            });

        } else if(gamepad.a) {
            collector.flipCollectorBox(0.6);
            collector.liftDepo(2000, false, new EventCallback() {
                @Override
                public void callbackEvent() {
                    robotLatch.teleUnhook();
                }
            });
        } else if(gamepad.b) {
            robotLatch.autoHook();
            sleep(500);
            collector.lowerDepo();
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
        // intake box controls
        if(gamepad.x) {
            // lower
            collector.flipCollectorBox(0.9);
        } else if(gamepad.a) {
            // prepare
            collector.flipCollectorBox(0.6);
        } else if (gamepad.b){
            // deposit
            collector.flipCollectorBox(0.19);
        } else if(gamepad.right_trigger > 0d) {
            collector.jerkCollectorBox();
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
            collector.liftDepo(2600, true);
        } else if(gamepad.dpad_down) {
            collector.lowerDepo();
        } else if(gamepad.right_bumper && collector.canFlipDepoBox()) {
            collector.rotateDepositBox(0.9, 1d);
        } else if(gamepad.left_bumper && collector.canFlipDepoBox()) {
            collector.jerkDepositBox();
        } else if(!collector.canFlipDepoBox()){
            collector.getDepo().setPosition(0.195);
        }
    }
}
