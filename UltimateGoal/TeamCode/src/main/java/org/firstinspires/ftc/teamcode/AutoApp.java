package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.detectors.StarterRingsDetector;
import org.firstinspires.ftc.teamcode.robot.AutoRobot;
import org.firstinspires.ftc.teamcode.util.AutonomousColor;
import org.firstinspires.ftc.teamcode.util.OpModeStore;
import org.firstinspires.ftc.teamcode.util.OpModeUtils;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

public abstract class AutoApp extends LinearOpMode {
    protected AutonomousColor autoColor = AutonomousColor.Unknown;

    protected AutoRobot robot;
    protected StarterRingsDetector detector;

    @Override
    public void runOpMode() throws InterruptedException {
        // Tell global store that runMode is in Autonomous mode
        OpModeUtils.getGlobalStore().runMode = OpModeStore.RunMode.Autonomous;

        // Tell global store the color of alliance
        OpModeUtils.getGlobalStore().autoColor = autoColor;

        // Reset encoder
        OpModeUtils.setResetEncoder(true);

        // Init ThreadUtils
        ThreadUtils.init();

        // Init OpModeUtils
        OpModeUtils.init(this);

        robot = new AutoRobot();
        robot.init();

        detector = new StarterRingsDetector();
        detector.init();
        detector.activate();

        // wait for player to hit star
        waitForStart();

        if(isStopRequested()) {
            return;
        }

        // Invoke Robot Autonomous Operations
        performRobotOperation();

        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = robot.getDrive().getPoseEstimate();
    }

    public abstract void performRobotOperation() throws InterruptedException;
}
