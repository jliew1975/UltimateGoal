package org.firstinspires.ftc.teamcode.team12538;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.team12538.ext.AutoGamepad;
import org.firstinspires.ftc.teamcode.team12538.detectors.TargetPositionalDetector;
import org.firstinspires.ftc.teamcode.team12538.detectors.opencv.OpenCvDetector;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive.AutoDirection;
import org.firstinspires.ftc.teamcode.team12538.robot.SkyStoneAutoRobot;
import org.firstinspires.ftc.teamcode.team12538.utils.AutoGamepadUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousColor;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeStore;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

public abstract class AutoLoadingZoneApp extends RobotApp {
    protected ElapsedTime runtime = new ElapsedTime();

    @Override
    public void performRobotOperation() throws InterruptedException {
        // Tell global store that runMode is in Autonomous mode
        OpModeUtils.getGlobalStore().runMode = OpModeStore.RunMode.Autonomous;

        // Tell global store the color of alliance
        OpModeUtils.getGlobalStore().autoColor = autoColor;

        // Reset encoder
        OpModeUtils.setResetEncoder(true);

        // Initialize a autonomous gamepad
        gamepad = new AutoGamepad();

        robot = new SkyStoneAutoRobot();
        robot.init();

        OpenCvDetector detector = new OpenCvDetector(numSkystone);
        detector.init();
        detector.activate();

        waitForStart();

        try {
            autoVisionLogic(detector);
        } finally {
            detector.deactivate();
            robot.intakeSensor.stop();
        }
    }

    protected abstract void autoVisionLogic(TargetPositionalDetector detector);
}
