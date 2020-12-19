package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.components.Shooter;
import org.firstinspires.ftc.teamcode.detectors.StarterRingsDetector;
import org.firstinspires.ftc.teamcode.robot.AutoRobot;
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory;
import org.firstinspires.ftc.teamcode.util.OpModeStore;
import org.firstinspires.ftc.teamcode.util.OpModeUtils;
import org.firstinspires.ftc.teamcode.util.GlobalStorage;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

public abstract class AutoApp extends CommonOpMode {
    protected AutoRobot robot;
    protected StarterRingsDetector detector;

    protected TrajectoryFactory trajectoryFactory;

    public AutoApp() {
        initTargetPoseValues();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Tell global store that runMode is in Autonomous mode
        OpModeUtils.getGlobalStore().runMode = OpModeStore.RunMode.Autonomous;

        // Reset encoder
        OpModeUtils.setResetEncoder(true);

        // Init ThreadUtils
        ThreadUtils.init();

        // Init OpModeUtils
        OpModeUtils.init(this);

        robot = new AutoRobot();
        robot.init();

        trajectoryFactory.init(robot);

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
        GlobalStorage.currentPose = robot.getDrive().getPoseEstimate();

    }

    public abstract void performRobotOperation() throws InterruptedException;

    protected void shootPowerShot() {
        Shooter shooter = robot.get(Shooter.class);
        shooter.start();

        try {
            robot.getDrive().turn(calculatePowerShotAngle(robot, PowerShotPos.One));
            shooter.liftShooter(pShotPose1, -0.004);
            ThreadUtils.sleep(500);
            shooter.fireSync();
            robot.getDrive().turn(calculatePowerShotAngle(robot, PowerShotPos.Two));
            shooter.liftShooter(pShotPose2, -0.004);
            ThreadUtils.sleep(500);
            shooter.fireSync();
            robot.getDrive().turn(calculatePowerShotAngle(robot, PowerShotPos.Three));
            shooter.liftShooter(pShotPose3, -0.004);
            ThreadUtils.sleep(500);
            shooter.fireSync();
        } finally {
            shooter.stop();
        }
    }
}
