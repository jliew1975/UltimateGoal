package org.firstinspires.ftc.teamcode.team12538;

import org.firstinspires.ftc.teamcode.team12538.components.AutoGamepad;
import org.firstinspires.ftc.teamcode.team12538.components.RobotStoneArm;
import org.firstinspires.ftc.teamcode.team12538.detectors.vuforia.VuforiaDetector;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive.AutoDirection;
import org.firstinspires.ftc.teamcode.team12538.robot.SkyStoneAutoRobot;
import org.firstinspires.ftc.teamcode.team12538.utils.AutoColor;
import org.firstinspires.ftc.teamcode.team12538.utils.AutoGamepadUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeStore;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

public class AutoLoadingZoneApp extends RobotApp {
    private AutoGamepad gamepad = null;
    private SkyStoneAutoRobot robot = null;

    @Override
    public void performRobotOperation() throws InterruptedException {
        // Tell global store that runMode is in Autonomous mode
        OpModeUtils.getGlobalStore().runMode = OpModeStore.RunMode.Autonomous;

        // Reset encoder
        OpModeUtils.setResetEncoder(true);

        // Initialize a autonomous gamepad
        gamepad = new AutoGamepad();

        robot = new SkyStoneAutoRobot();
        robot.init();

        VuforiaDetector detector = new VuforiaDetector(autoColor);
        detector.init();
        // detector.activate();
        detector.activate(VuforiaDetector.TargetMode.StoneDetection);

        waitForStart();

        try {
            autoVuforiaLogic(detector);
        } finally {
            detector.deactivate();
        }
    }

    private void autoVuforiaLogic(VuforiaDetector detector) {
        AutoGamepadUtils.move(gamepad, AutoDirection.Forward, 0.3, 16);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.outtake.liftSlideForStoneIntake();

        // wait for 1 second for vuforia to pickup skystone position.
        sleep(1000);
        VuforiaDetector.TargetPosition position = detector.getTargetPosition();

        // enable intake rollers
        robot.intake.setPower(0.6);

        switch (position) {
            case Left:
                executeLeftLogic();
                break;
            case Center:
                executeCenterLogic();
                break;
            default:
                executeRightLogic();
        }

        sleep(500);
    }

    private void autoArmLogic() {
        AutoGamepadUtils.move(gamepad, resolveDirectionForArm(AutoDirection.StrafeLeft), 0.3, 30);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        robot.autoStoneArm.setPosition(RobotStoneArm.DOWN);

        sleep(300);

        AutoGamepadUtils.move(gamepad, resolveDirectionForArm(AutoDirection.StrafeRight), 0.3, 13);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, resolveDirectionForArm(AutoDirection.Forward), 0.5, 40);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.autoStoneArm.setPosition(RobotStoneArm.UP);
        sleep(300);

        AutoGamepadUtils.move(gamepad, resolveDirectionForArm(AutoDirection.Backward), 0.5, 10);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        gamepad.timeout = 3d;
        AutoGamepadUtils.move(gamepad, resolveDirectionForArm(AutoDirection.StrafeLeft), 0.3, 10);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, resolveDirectionForArm(AutoDirection.StrafeRight), 0.3, 5);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, resolveDirectionForArm(AutoDirection.Backward), 0.5, 40);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.turn(gamepad, resolveDirectionForArm(AutoDirection.TurnRight), 0.2, 10);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        AutoGamepadUtils.move(gamepad, resolveDirectionForArm(AutoDirection.StrafeLeft), 0.3, 12);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.autoStoneArm.setPosition(RobotStoneArm.DOWN);

        sleep(500);

        AutoGamepadUtils.move(gamepad, resolveDirectionForArm(AutoDirection.StrafeRight), 0.3, 13);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, resolveDirectionForArm(AutoDirection.Forward), 0.5, 50);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.autoStoneArm.setPosition(RobotStoneArm.UP);
        sleep(300);

        AutoGamepadUtils.move(gamepad, resolveDirectionForArm(AutoDirection.Backward), 0.5, 10);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        AutoGamepadUtils.move(gamepad, resolveDirectionForArm(AutoDirection.StrafeLeft), 0.3, 5d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

    }

    private void executeLeftLogic() {
        // Position our robot in an angle due to our roller intake limitation
        AutoGamepadUtils.move(gamepad, resolveDirectionForVuforia(AutoDirection.StrafeRight), 0.3, 3d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        pickupStoneNavigation();

        sleep(500);

        // At this point we assume we have the stone in the robot, so we need to stop the intake and
        // position the outtake mechanism for stone deplyment.
        AutoGamepadUtils.move(gamepad, resolveDirectionForVuforia(AutoDirection.Backward), 0.3, 10d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.intake.setPower(0d);
        robot.outtake.lowerSlideForStonePickup();

        // Start of navigation to Building site logic
        AutoGamepadUtils.turn(gamepad, resolveDirectionForVuforia(AutoDirection.TurnLeft), 0.5, 40d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, resolveDirectionForVuforia(AutoDirection.Backward), 0.5, 50d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.outtake.prepareForStoneDeployment();

        sleep(500);

        robot.outtake.performOuttakeClawOperation();

        sleep(500);

        AutoGamepadUtils.move(gamepad, resolveDirectionForVuforia(AutoDirection.Forward), 0.5, 5d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.outtake.performStonePickupOperation();

        sleep(500);

        // Start of Parking under the Skybridge logic
        AutoGamepadUtils.move(gamepad, resolveDirectionForVuforia(AutoDirection.Forward), 0.5, 10d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
    }

    public void executeCenterLogic() {
        // Position our robot in an angle due to our roller intake limitation
        AutoGamepadUtils.move(gamepad, resolveDirectionForVuforia(AutoDirection.StrafeRight), 0.3, 10d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        pickupStoneNavigation();

        sleep(500);

        // At this point we assume we have the stone in the robot, so we need to stop the intake and
        // position the outtake mechanism for stone deplyment.
        AutoGamepadUtils.move(gamepad, resolveDirectionForVuforia(AutoDirection.Backward), 0.3, 10d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.intake.setPower(0d);
        robot.outtake.lowerSlideForStonePickup();

        // Start of navigation to Building site logic
        AutoGamepadUtils.turn(gamepad, resolveDirectionForVuforia(AutoDirection.TurnLeft), 0.5, 40d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, resolveDirectionForVuforia(AutoDirection.Backward), 0.5, 50d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.outtake.prepareForStoneDeployment();

        sleep(500);

        robot.outtake.performOuttakeClawOperation();

        sleep(500);

        AutoGamepadUtils.move(gamepad, resolveDirectionForVuforia(AutoDirection.Forward), 0.5, 5d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.outtake.performStonePickupOperation();

        sleep(500);

        // Start of Parking under the Skybridge logic
        AutoGamepadUtils.move(gamepad, resolveDirectionForVuforia(AutoDirection.Forward), 0.5, 10d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
    }

    public void executeRightLogic() {
        // Position our robot in an angle due to our roller intake limitation
        AutoGamepadUtils.move(gamepad, resolveDirectionForVuforia(AutoDirection.StrafeRight), 0.3, 20d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        pickupStoneNavigation();

        sleep(500);

        // At this point we assume we have the stone in the robot, so we need to stop the intake and
        // position the outtake mechanism for stone deplyment.
        AutoGamepadUtils.move(gamepad, resolveDirectionForVuforia(AutoDirection.Backward), 0.3, 15d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.intake.setPower(0d);
        robot.outtake.lowerSlideForStonePickup();

        AutoGamepadUtils.turn(gamepad, resolveDirectionForVuforia(AutoDirection.TurnLeft), 0.5, 40d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, resolveDirectionForVuforia(AutoDirection.Backward), 0.5, 40d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.outtake.prepareForStoneDeployment();

        sleep(500);

        robot.outtake.performOuttakeClawOperation();

        sleep(500);

        AutoGamepadUtils.move(gamepad, resolveDirectionForVuforia(AutoDirection.Forward), 0.5, 5d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.outtake.performStonePickupOperation();

        sleep(500);

        // Start of Parking under the Skybridge logic
        AutoGamepadUtils.move(gamepad, resolveDirectionForVuforia(AutoDirection.Forward), 0.5, 10d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
    }

    private void pickupStoneNavigation() {
        // TODO: Need sensor to know if we have stone in the robot.
        // Currently we assume the stone is in.
        AutoGamepadUtils.turn(gamepad, resolveDirectionForVuforia(AutoDirection.TurnLeft), 0.2, 15d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, resolveDirectionForVuforia(AutoDirection.Forward), 0.3, 25d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.turn(gamepad, resolveDirectionForVuforia(AutoDirection.TurnLeft), 0.2, 20d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
    }

    private MecanumDrive.AutoDirection resolveDirectionForVuforia(MecanumDrive.AutoDirection currentDirection) {
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

    private MecanumDrive.AutoDirection resolveDirectionForArm(MecanumDrive.AutoDirection currentDirection) {
        if(autoColor == AutoColor.Blue) {
            switch(currentDirection) {
                case Forward:
                case Backward:
                    return flipDirection(currentDirection);
            }
        }

        return currentDirection;
    }
}
