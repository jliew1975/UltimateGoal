package org.firstinspires.ftc.teamcode.team12538;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.team12538.components.AutoGamepad;
import org.firstinspires.ftc.teamcode.team12538.components.RobotDistanceSensor;
import org.firstinspires.ftc.teamcode.team12538.detectors.RobotDetectorLimit;
import org.firstinspires.ftc.teamcode.team12538.detectors.vuforia.VuforiaDetector;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive.AutoDirection;
import org.firstinspires.ftc.teamcode.team12538.robot.SkyStoneAutoRobot;
import org.firstinspires.ftc.teamcode.team12538.utils.AutoGamepadUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousColor;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeStore;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

public abstract class AutoLoadingZoneApp extends RobotApp {
    protected AutoGamepad gamepad = null;
    protected SkyStoneAutoRobot robot = null;

    protected ElapsedTime runtime = new ElapsedTime();

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

        VuforiaDetector detector = new VuforiaDetector();
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

    protected abstract void autoVuforiaLogic(VuforiaDetector detector);

    protected void pickupStoneNavigation() {
        pickupStoneNavigation(10d);
    }

    protected void pickupStoneNavigation(double distance) {
        gamepad.detector = robot.intakeSensor;
        AutoGamepadUtils.move(gamepad, resolveDirectionForVuforia(AutoDirection.Forward), 0.2, distance);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
    }

    protected MecanumDrive.AutoDirection resolveDirectionForVuforia(MecanumDrive.AutoDirection currentDirection) {
        if (autoColor == AutonomousColor.Blue) {
            switch (currentDirection) {
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
