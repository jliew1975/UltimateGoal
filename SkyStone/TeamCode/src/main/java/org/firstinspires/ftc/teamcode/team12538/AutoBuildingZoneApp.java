package org.firstinspires.ftc.teamcode.team12538;

import org.firstinspires.ftc.teamcode.team12538.components.AutoGamepad;
import org.firstinspires.ftc.teamcode.team12538.components.RobotFoundationClaw;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive.AutoDirection;
import org.firstinspires.ftc.teamcode.team12538.robot.SkyStoneAutoRobot;
import org.firstinspires.ftc.teamcode.team12538.utils.AutoColor;
import org.firstinspires.ftc.teamcode.team12538.utils.AutoGamepadUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeStore;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

public class AutoBuildingZoneApp extends RobotApp {
    @Override
    public void performRobotOperation() throws InterruptedException {
        // Tell global store that runMode is in Autonomous mode
        OpModeUtils.getGlobalStore().runMode = OpModeStore.RunMode.Autonomous;

        // reset encoder
        OpModeUtils.setResetEncoder(true);

        // Initialize a autonomous gamepad
        AutoGamepad gamepad = new AutoGamepad();

        SkyStoneAutoRobot robot = new SkyStoneAutoRobot();
        robot.init();

        waitForStart();

        // Navigate halfway to foundation
        AutoGamepadUtils.move(gamepad, resolveDirection(AutoDirection.Backward), 0.7,15);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        AutoGamepadUtils.move(gamepad, resolveDirection(AutoDirection.StrafeLeft), 0.8,10);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        // Navigate the to foundation
        AutoGamepadUtils.move(gamepad, resolveDirection(AutoDirection.Backward), 0.7,15);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        // Position foundation claw to pull foundation
        robot.foundationClaw.setClawPosition(RobotFoundationClaw.LOWER_CLAW_POS);
        sleep(500);

        // Pull foundation to building site
        AutoGamepadUtils.move(gamepad, resolveDirection(AutoDirection.Forward), 0.7,40);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        // Release foundation by lifting foundation claw
        robot.foundationClaw.setClawPosition(RobotFoundationClaw.INIT_POSITION);
        sleep(500);

        AutoGamepadUtils.move(gamepad, resolveDirection(AutoDirection.StrafeRight), 0.8,50);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
    }

    private MecanumDrive.AutoDirection resolveDirection(MecanumDrive.AutoDirection currentDirection) {
        if(autoColor == AutoColor.Blue) {
            switch(currentDirection) {
                case StrafeLeft:
                case StrafeRight:
                    return flipDirection(currentDirection);
                case TurnRight:
                case TurnLeft:
                    return flipDirection(currentDirection);
            }
        }

        return currentDirection;
    }
}
