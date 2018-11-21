package org.firstinspires.ftc.teamcode.team12538.robotV1;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.team12538.components.MineralMechanism;
import org.firstinspires.ftc.teamcode.team12538.components.MineralMechanism.MineralSide;

public class TeleOpRobotV1 extends RobotBase {
    private boolean isLatched = false;

    @Override
    public void init() {
        super.init();
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
            collector.enableIntake(MineralMechanism.Direction.InTake);
        } else if(gamepad.left_bumper) {
            collector.enableIntake(MineralMechanism.Direction.OutTake);
        } else {
            if(!collector.isIntakeAutoOn()) {
                collector.disableIntake();
            }
        }

        if(gamepad.x) {
            robotLatch.teleHook();
        } else if(gamepad.a) {
            robotLatch.teleUnhook();
        } else if(gamepad.y) {
            robotLatch.powerLift(-1.0);
        }

        if (gamepad.dpad_up) {
            robotLatch.powerLift(1.0);
        } else if (gamepad.dpad_down) {
            robotLatch.powerLift(-1.0, 100);
        } else {
            robotLatch.powerLift(0d);
        }
    }

    public void player2Controls(Gamepad gamepad) {
        if(gamepad.x) {
            // lower
            collector.flipCollectorBox(0d);
            // collector.enableIntake(MineralMechanism.Direction.InTake, true);
        } else if(gamepad.a) {
            // prepare
            collector.flipCollectorBox(0.6);
        } else if (gamepad.b){
            // deposit
            collector.flipCollectorBox(1d);
        }

        if(gamepad.y) {
            // auto deposit minerals to outtake box
            collector.autoMineralDeposit();
        } else {
            // arm extension control
            collector.controlArm(-gamepad.left_stick_x);
        }

        // swinging arm control
        if(gamepad.left_bumper) {
            collector.swingArmToPosition(450, 0.15);
        } else if(gamepad.right_bumper) {
            collector.swingArmToPosition(80, 0.08);
            collector.swingArmToPosition(0, 0.08);
        }

        if(collector.isSwingArmUp()) {
            if (gamepad.right_trigger > 0) {
                collector.controlReleaseMineral(MineralSide.Left, 0d);
            } else {
                collector.controlReleaseMineral(MineralSide.Left, 0.5);
            }

            if (gamepad.left_trigger > 0) {
                collector.controlReleaseMineral(MineralSide.Right, 0d);
            } else {
                collector.controlReleaseMineral(MineralSide.Right, 0.5);
            }
        } else {
            collector.controlReleaseMineral(MineralSide.Both, 0d);
        }

        // adjustment control
        if(gamepad.dpad_up) {
            collector.swingArmPositionBy(20);
        } else if(gamepad.dpad_down) {
            collector.swingArmPositionBy(-20);
        } else if(gamepad.dpad_right) {
            collector.adjustArmPosition(-10, true);
        } else if(gamepad.dpad_left) {
            collector.adjustArmPosition(10, false);
        }
    }
}
