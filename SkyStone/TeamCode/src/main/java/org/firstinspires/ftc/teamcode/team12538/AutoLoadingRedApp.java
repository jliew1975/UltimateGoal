package org.firstinspires.ftc.teamcode.team12538;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team12538.components.RobotStoneArm;
import org.firstinspires.ftc.teamcode.team12538.detectors.vuforia.VuforiaDetector;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.utils.AutoGamepadUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousColor;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousMode;

@Autonomous(name="Loading Red", group="Linear Opmode")
public class AutoLoadingRedApp extends AutoLoadingZoneApp {
    public AutoLoadingRedApp() {
        super();
        super.autoMode = AutonomousMode.RedLoading;
        super.autoColor = AutonomousColor.Red;
    }

    @Override
    protected void autoVuforiaLogic(VuforiaDetector detector) {
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 22);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        // Default to left
        VuforiaDetector.TargetPosition targetPosition = VuforiaDetector.TargetPosition.Right;
        detector.targetPosition = targetPosition;

        // wait for 2 second for vuforia skystone detection.
        runtime.reset();
        detector.wait(2, runtime);

        try {
            if (!detector.isTargetVisible()) {
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 6);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

                // wait for 1 second for vuforia to pickup skystone position.
                runtime.reset();
                detector.wait(2, runtime);

                if (detector.isTargetVisible()) {
                    targetPosition = VuforiaDetector.TargetPosition.Center;
                    detector.targetPosition = targetPosition;
                } else {
                    targetPosition = VuforiaDetector.TargetPosition.Left;
                    detector.targetPosition = targetPosition;
                }
            }
        } finally {
            detector.deactivate();
        }

        switch (targetPosition) {
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

    protected void executeLeftLogic() {
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 1d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 12d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.stoneArm.setPosition(RobotStoneArm.DOWN);

        sleep(800);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 15d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        // Start of navigation to Building site logic
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 53d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.stoneArm.setPosition(RobotStoneArm.UP);

        // Start of navigation to Building site logic
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 10d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        // Start of navigation to Building site logic
        gamepad.timeout = 1d;
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 15d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 2d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.5, 30d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.2, 20d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.rightDistSensor.setLimit(4d);
        gamepad.detector = robot.leftDistSensor;
        gamepad.timeout = 2d;
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.2, 9d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 4d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        gamepad.timeout = 1d;
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 25d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.stoneArm.setPosition(RobotStoneArm.DOWN);

        sleep(500);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 15d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        // Start of Parking under the Skybridge logic
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 10d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.5, 20d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 45d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.stoneArm.setPosition(RobotStoneArm.UP);

        // Start of Parking under the Skybridge logic
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.5, 8d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 5d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
    }

    protected void executeCenterLogic() {
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 12d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.stoneArm.setPosition(RobotStoneArm.DOWN);

        sleep(800);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 15d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        // Start of navigation to Building site logic
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 45d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.stoneArm.setPosition(RobotStoneArm.UP);

        // Start of navigation to Building site logic
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 10d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        // Start of navigation to Building site logic
        gamepad.timeout = 1d;
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 15d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 2d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 30d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.5, 20d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.rightDistSensor.setLimit(4d);
        gamepad.detector = robot.leftDistSensor;
        gamepad.timeout = 2d;
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.2, 9d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 4d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        gamepad.timeout = 1d;
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 25d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.stoneArm.setPosition(RobotStoneArm.DOWN);

        sleep(500);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 15d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        // Start of Parking under the Skybridge logic
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 10d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.5, 20d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 40d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.stoneArm.setPosition(RobotStoneArm.UP);

        // Start of Parking under the Skybridge logic
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.5, 8d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 5d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
    }

    protected void executeRightLogic() {
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 6.2d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 10d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.stoneArm.setPosition(RobotStoneArm.DOWN);

        sleep(500);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 15d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        // Start of navigation to Building site logic
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 45d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.stoneArm.setPosition(RobotStoneArm.UP);

        // Start of navigation to Building site logic
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 8d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        // Start of navigation to Building site logic
        gamepad.timeout = 1d;
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 15d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 2d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 30d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.5, 20d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.rightDistSensor.setLimit(4d);
        gamepad.detector = robot.leftDistSensor;
        gamepad.timeout = 3d;
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.2, 9d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 10d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        gamepad.timeout = 1d;
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 25d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.stoneArm.setPosition(RobotStoneArm.DOWN);

        sleep(500);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 15d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        // Start of Parking under the Skybridge logic
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 10d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.5, 20d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 35d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.stoneArm.setPosition(RobotStoneArm.UP);

        // Start of Parking under the Skybridge logic
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.5, 5d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 5d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
    }
}
