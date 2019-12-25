package org.firstinspires.ftc.teamcode.team12538;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team12538.components.RobotStoneArm;
import org.firstinspires.ftc.teamcode.team12538.components.RobotStoneClaw;
import org.firstinspires.ftc.teamcode.team12538.detectors.TargetPositionalDetector;
import org.firstinspires.ftc.teamcode.team12538.detectors.vuforia.VuforiaDetector;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.utils.AutoGamepadUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousColor;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousMode;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

@Autonomous(name="Loading Red", group="Linear Opmode")
public class AutoLoadingRedApp extends AutoLoadingZoneApp {
    public AutoLoadingRedApp() {
        super();
        super.autoMode = AutonomousMode.RedLoading;
        super.autoColor = AutonomousColor.Red;
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
        pickupSkyStone();

        crossSkyBridge();

        moveFoundationToBuildingSite();

        deploySkyStoneToFoundation();

        // moveToParkUnderSkyBridge();
    }

    private void pickupSkyStone() {
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 20d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(200);

        // rotate robot for stone intake
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.TurnLeft, 0.3, Math.PI/2);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(200);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 5d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(200);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 18d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        ThreadUtils.getExecutorService().submit(() -> robot.outtake.outtakeSlides.runToPosition(40));

        sleep(200);

        // enable intake
        robot.intake.setPower(1);

        // start the distance sensor for stone detection
        robot.intakeSensor.start();

        sleep(500);

        gamepad.detector = robot.intakeSensor;
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        ThreadUtils.getExecutorService().submit(() -> {
            robot.outtake.lowerSlideForStonePickup();
            robot.intake.setPower(0);
            robot.outtake.outtakeClaw.setArmPosition(RobotStoneClaw.CLAW_OPEN_POSITION);
            sleep(500);
            robot.outtake.outtakeClaw.setArmPosition(RobotStoneClaw.CLAW_CLOSE_POSITION);
        });
    }

    private void crossSkyBridge() {
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 12d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(200);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.5, 50d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 20d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);
    }

    private void moveFoundationToBuildingSite() {
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.TurnLeft, 0.3, Math.PI/2);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(200);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 8);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.foundationClaw.lowerClaw();

        sleep(800);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.6, 2);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(200);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.CurveRight, 0.6, 30);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(200);

        gamepad.resetAngle = true;
        gamepad.timeout = 1d;
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.5, 15);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.foundationClaw.raiseClaw();
    }

    private void deploySkyStoneToFoundation() {
        robot.outtake.prepareForStoneDeployment();

        sleep(1000);

        robot.outtake.outtakeSlides.runToPosition(20, true);

        sleep(500);

        robot.outtake.outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_OPEN_POSITION);

        sleep(500);

        robot.outtake.outtakeSlides.runToStoneHeight(robot.outtake.stoneHeight);

        sleep(500);

        robot.outtake.performStoneIntakeOperation();
    }

    private void moveToParkUnderSkyBridge() {
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.5, 30);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
    }

    protected void executeCenterLogic() {
        /*
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 12d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(800);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 15d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        // Start of navigation to Building site logic
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 45d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

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

        // Start of Parking under the Skybridge logic
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.5, 8d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 5d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
        */
    }

    protected void executeRightLogic() {
        /*
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 6.2d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 10d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(500);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 15d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        // Start of navigation to Building site logic
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 45d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

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

        // Start of Parking under the Skybridge logic
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.5, 5d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 5d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
        */
    }
}
