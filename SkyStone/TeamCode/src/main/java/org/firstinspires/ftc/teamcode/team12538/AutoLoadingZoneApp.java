package org.firstinspires.ftc.teamcode.team12538;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.team12538.components.AutoGamepad;
import org.firstinspires.ftc.teamcode.team12538.components.RobotOuttakeSlides;
import org.firstinspires.ftc.teamcode.team12538.detectors.vuforia.VuforiaDetector;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive.AutoDirection;
import org.firstinspires.ftc.teamcode.team12538.robot.SkyStoneAutoRobot;
import org.firstinspires.ftc.teamcode.team12538.utils.AutoGamepadUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousColor;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeStore;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.RobotUtils;

public class AutoLoadingZoneApp extends RobotApp {
    private AutoGamepad gamepad = null;
    private SkyStoneAutoRobot robot = null;

    private ElapsedTime runtime = new ElapsedTime();

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

    private void autoVuforiaLogic(VuforiaDetector detector) {
        AutoGamepadUtils.move(gamepad, AutoDirection.Forward, 0.3, 20);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        // wait for 1 second for vuforia to pickup skystone position.
        sleep(2000);

        detector.targetPosition = VuforiaDetector.TargetPosition.Left;
        if(!detector.isTargetVisible()) {
            AutoGamepadUtils.move(gamepad, AutoDirection.StrafeRight, 0.3, 7);
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

    private void executeLeftLogic() {
        // Position our robot in an angle due to our roller intake limitation
        AutoGamepadUtils.move(gamepad, resolveDirectionForVuforia(AutoDirection.StrafeRight), 0.3, 10d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        pickupStoneNavigation();

        sleep(500);

        // At this point we assume we have the stone in the robot, so we need to stop the intake and
        // position the outtake mechanism for stone deplyment.
        AutoGamepadUtils.move(gamepad, resolveDirectionForVuforia(AutoDirection.Backward), 0.3, 10d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        RobotUtils.prepareForStonePickup(robot);

        // Start of navigation to Building site logic
        AutoGamepadUtils.turn(gamepad, resolveDirectionForVuforia(AutoDirection.TurnLeft), 0.5, 40d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, resolveDirectionForVuforia(AutoDirection.Backward), 0.5, 60d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        RobotUtils.deploySkyStone(robot);

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

        RobotUtils.prepareForStonePickup(robot);

        // Start of navigation to Building site logic
        AutoGamepadUtils.turn(gamepad, resolveDirectionForVuforia(AutoDirection.TurnLeft), 0.5, 50d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, resolveDirectionForVuforia(AutoDirection.Backward), 0.5, 50d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        RobotUtils.deploySkyStone(robot);

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

        RobotUtils.prepareForStonePickup(robot);

        AutoGamepadUtils.turn(gamepad, resolveDirectionForVuforia(AutoDirection.TurnLeft), 0.5, 40d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, resolveDirectionForVuforia(AutoDirection.Backward), 0.5, 40d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        RobotUtils.deploySkyStone(robot);

        AutoGamepadUtils.move(gamepad, resolveDirectionForVuforia(AutoDirection.Forward), 0.5, 5d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.outtake.performStonePickupOperation();

        sleep(500);

        // Start of Parking under the Skybridge logic
        AutoGamepadUtils.move(gamepad, resolveDirectionForVuforia(AutoDirection.Forward), 0.5, 10d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
    }

    private void pickupStoneNavigation() {
        AutoGamepadUtils.turn(gamepad, resolveDirectionForVuforia(AutoDirection.TurnLeft), 0.2, 20d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        gamepad.detector = robot.intakeSensor;
        AutoGamepadUtils.move(gamepad, resolveDirectionForVuforia(AutoDirection.Forward), 0.3, 30d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
    }

    private MecanumDrive.AutoDirection resolveDirectionForVuforia(MecanumDrive.AutoDirection currentDirection) {
        if(autoColor == AutonomousColor.Blue) {
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
        if(autoColor == AutonomousColor.Blue) {
            switch(currentDirection) {
                case Forward:
                case Backward:
                    return flipDirection(currentDirection);
            }
        }

        return currentDirection;
    }
}
