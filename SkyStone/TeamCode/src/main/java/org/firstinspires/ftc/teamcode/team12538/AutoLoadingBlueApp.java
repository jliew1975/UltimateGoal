package org.firstinspires.ftc.teamcode.team12538;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team12538.components.RobotStoneArm;
import org.firstinspires.ftc.teamcode.team12538.detectors.TargetPositionalDetector;
import org.firstinspires.ftc.teamcode.team12538.detectors.vuforia.VuforiaDetector;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.utils.AutoGamepadUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousColor;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousMode;
import org.firstinspires.ftc.teamcode.team12538.utils.RobotUtils;

@Autonomous(name="Loading Blue", group="Linear Opmode")
public class AutoLoadingBlueApp extends AutoLoadingZoneApp {
    public AutoLoadingBlueApp() {
        super();
        super.autoMode = AutonomousMode.BlueLoading;
        super.autoColor = AutonomousColor.Blue;
    }

    @Override
    protected void autoVuforiaLogic(TargetPositionalDetector detector) {
        TargetPositionalDetector.Position skystonePosition = detector.getPosition();

        switch (skystonePosition) {
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
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 6.5d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);


        sleep(500);


        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 10d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 16d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        // Start of navigation to Building site logic
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.5, 38d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        // Start of navigation to Building site logic
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.5, 10d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        // Start of navigation to Building site logic
        gamepad.timeout = 1d;
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 10d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 3d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);


        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 20d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.rightDistSensor.setLimit(4.0);
        gamepad.detector = robot.leftDistSensor;
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.2, 38d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.2, 9d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        gamepad.timeout = 2d;
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 20d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 15d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        gamepad.timeout = 1d;
        robot.rightDistSensor.setLimit(4.0);
        gamepad.detector = robot.leftDistSensor;
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 15d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(100);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.5, 30d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 45d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        // Start of Parking under the Skybridge logic
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 8d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
    }

    protected void executeCenterLogic() {
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 6.2);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 10d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 16d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        // Start of navigation to Building site logic
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 50d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        // Start of navigation to Building site logic
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        // Start of navigation to Building site logic
        gamepad.timeout = 1d;
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 15d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 2d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.5, 40d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.rightDistSensor.setLimit(4d);
        gamepad.detector = robot.leftDistSensor;
        gamepad.timeout = 3d;
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.2, 40d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 3d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        gamepad.timeout = 1d;
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 20d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 15d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        // Start of Parking under the Skybridge logic
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.5, 35d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 39d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        // Start of Parking under the Skybridge logic
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.5, 5d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 5d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
    }

    protected void executeRightLogic() {
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 15d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 10d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 16d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        // Start of navigation to Building site logic
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 60d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        // Start of navigation to Building site logic
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        // Start of navigation to Building site logic
        gamepad.timeout = 1d;
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 15d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 2d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.5, 40d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.rightDistSensor.setLimit(4d);
        gamepad.detector = robot.leftDistSensor;
        gamepad.timeout = 3d;
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.2, 40d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 3d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        gamepad.timeout = 1d;
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 20d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 15d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        // Start of Parking under the Skybridge logic
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.5, 35d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 39d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        // Start of Parking under the Skybridge logic
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.5, 5d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 5d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
    }
}
