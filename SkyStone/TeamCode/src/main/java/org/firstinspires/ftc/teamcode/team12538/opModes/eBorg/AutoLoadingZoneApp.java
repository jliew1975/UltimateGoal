package org.firstinspires.ftc.teamcode.team12538.opModes.eBorg;

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
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;
import org.openftc.revextensions2.ExpansionHubEx;

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

        ThreadUtils.getExecutorService().submit(() -> detector.deactivate());

        try {
            if(!isStopRequested()) {
                autoVisionLogic(detector);

                while(opModeIsActive()) {
                    ThreadUtils.idle();
                }
            }
        } finally {
            robot.intakeSensor.stop();
            OpModeUtils.stop();
        }
    }

    protected abstract void autoVisionLogic(TargetPositionalDetector detector);
}
