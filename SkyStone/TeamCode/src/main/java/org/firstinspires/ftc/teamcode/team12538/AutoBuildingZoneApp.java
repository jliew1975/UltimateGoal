package org.firstinspires.ftc.teamcode.team12538;

import org.firstinspires.ftc.teamcode.team12538.ext.AutoGamepad;
import org.firstinspires.ftc.teamcode.team12538.components.RobotDistanceSensor;
import org.firstinspires.ftc.teamcode.team12538.components.RobotFoundationClaw;
import org.firstinspires.ftc.teamcode.team12538.detectors.RobotDetectorLimit;
import org.firstinspires.ftc.teamcode.team12538.detectors.vuforia.VuforiaDetector;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive.AutoDirection;
import org.firstinspires.ftc.teamcode.team12538.robot.SkyStoneAutoRobot;
import org.firstinspires.ftc.teamcode.team12538.utils.AutoGamepadUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousColor;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeStore;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

public class AutoBuildingZoneApp extends RobotApp {
    protected boolean pickupSkystone = false;
    protected VuforiaDetector detector = null;


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

        if(pickupSkystone) {
            detector = new VuforiaDetector();
            detector.init();
            detector.activate(VuforiaDetector.TargetMode.StoneDetection);
        }

        RobotDistanceSensor leftDistanceSensor = robot.leftDistSensor;
        RobotDistanceSensor rightDistanceSensor = robot.rightDistSensor;

        RobotDetectorLimit sideViewDetector = (autoColor == AutonomousColor.Blue) ? leftDistanceSensor : rightDistanceSensor;
        ((RobotDistanceSensor)sideViewDetector).sideView();

        RobotDetectorLimit frontViewDetector = (autoColor == AutonomousColor.Blue) ? rightDistanceSensor : leftDistanceSensor;
        ((RobotDistanceSensor)frontViewDetector).frontView();

        waitForStart();

        // Navigate halfway to foundation
        AutoGamepadUtils.move(gamepad, resolveDirection(AutoDirection.Backward), 0.5,15);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        AutoGamepadUtils.move(gamepad, resolveDirection(AutoDirection.StrafeLeft), 0.5,10);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        // Navigate the to foundation
        AutoGamepadUtils.move(gamepad, resolveDirection(AutoDirection.Backward), 0.5,15);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        // Position foundation claw to pull foundation
        robot.foundationClaw.setClawPosition(RobotFoundationClaw.LOWER_CLAW_POS);
        sleep(500);

        // Pull foundation to building site
        frontViewDetector.setLimit(0d);
        gamepad.detector = frontViewDetector;
        AutoGamepadUtils.move(gamepad, resolveDirection(AutoDirection.Forward), 0.5,38);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        // Release foundation by lifting foundation claw
        robot.foundationClaw.setClawPosition(RobotFoundationClaw.INIT_POSITION);
        sleep(500);

        sideViewDetector.setLimit(3d);
        gamepad.detector = sideViewDetector;
        AutoGamepadUtils.move(gamepad, resolveDirection(AutoDirection.StrafeRight), 0.3,50);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        if(sideViewDetector.isDetected()) {
            AutoGamepadUtils.move(gamepad, resolveDirection(AutoDirection.Backward), 0.3,18);
            robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

            sleep(500);

            AutoGamepadUtils.move(gamepad, resolveDirection(AutoDirection.StrafeRight), 0.3,18);
            robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

            if(pickupSkystone) {
                AutoGamepadUtils.move(gamepad, resolveDirection(AutoDirection.StrafeRight), 0.3,25);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

                AutoGamepadUtils.move(gamepad, resolveDirection(AutoDirection.Forward), 0.3,10);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

                detector.targetPosition = VuforiaDetector.TargetPosition.Left;
                if(!detector.isTargetVisible()) {
                    AutoGamepadUtils.move(gamepad, resolveDirection(AutoDirection.StrafeLeft), 0.3, 7);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

                    // wait for 1 second for vuforia to pickup skystone position.
                    sleep(1000);
                    if(detector.isTargetVisible()) {
                        detector.targetPosition = VuforiaDetector.TargetPosition.Center;
                    } else {
                        detector.targetPosition = VuforiaDetector.TargetPosition.Right;
                    }
                }

                robot.outtake.liftSlideForStoneIntake();

                // enable intake rollers
                robot.intake.setPower(0.6);

                switch (detector.targetPosition) {
                    case Left:
                        // executeLeftLogic();
                        break;
                    case Center:
                        // executeCenterLogic();
                        break;
                    default:
                        // executeRightLogic();
                }
            }
        }

        sleep(500);
    }

    private MecanumDrive.AutoDirection resolveDirection(MecanumDrive.AutoDirection currentDirection) {
        if(autoColor == AutonomousColor.Blue) {
            switch(currentDirection) {
                case StrafeLeft:
                case StrafeRight:
                case TurnRight:
                case TurnLeft:
                case CurveRight:
                case CurveLeft:
                    return flipDirection(currentDirection);
            }
        }

        return currentDirection;
    }
}
