package org.firstinspires.ftc.teamcode.team12538;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team12538.components.RobotStoneClaw;
import org.firstinspires.ftc.teamcode.team12538.detectors.TargetPositionalDetector;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.utils.AutoGamepadUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousColor;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousMode;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

@Autonomous(name="Blue Skystone And Foundation", group="Linear Opmode")
public class AutoLoadingBlueStoneAndFoundationApp extends AutoLoadingZoneApp {
    public AutoLoadingBlueStoneAndFoundationApp() {
        super();
        super.autoMode = AutonomousMode.BlueLoading;
        super.autoColor = AutonomousColor.Blue;
    }

    @Override
    protected void autoVisionLogic(TargetPositionalDetector detector) {
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

    protected void executeRightLogic() {
        pickupSkyStone(TargetPositionalDetector.Position.Right);

        crossSkyBridge(TargetPositionalDetector.Position.Right);

        moveFoundationToBuildingSite(TargetPositionalDetector.Position.Right);

        // deploySkyStoneToFoundation();

        moveToParkUnderSkyBridge(TargetPositionalDetector.Position.Right);
    }

    protected void executeCenterLogic() {
        pickupSkyStone(TargetPositionalDetector.Position.Center);

        crossSkyBridge(TargetPositionalDetector.Position.Center);

        moveFoundationToBuildingSite(TargetPositionalDetector.Position.Center);

        // deploySkyStoneToFoundation();

        moveToParkUnderSkyBridge(TargetPositionalDetector.Position.Center);
    }

    protected void executeLeftLogic() {
        pickupSkyStone(TargetPositionalDetector.Position.Left);

        crossSkyBridge(TargetPositionalDetector.Position.Left);

        moveFoundationToBuildingSite(TargetPositionalDetector.Position.Left);

        // deploySkyStoneToFoundation();

        moveToParkUnderSkyBridge(TargetPositionalDetector.Position.Left);
    }

    private void pickupSkyStone(TargetPositionalDetector.Position position) {
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 10d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        // lift slide for intake in a background thread
        ThreadUtils.getExecutorService().submit(() -> robot.outtake.outtakeSlides.runToPosition(50));

        // enable intake
        robot.intake.setPower(1);

        // start the distance sensor for stone detection
        robot.intakeSensor.start();

        switch(position) {
            case Left:
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.CurveRight, 0.5, 10d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.3, 5.5d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.detector = robot.intakeSensor;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 15d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;

            case Center:
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 10);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.TurnRight, 0.3, Math.PI/2);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 2d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 14d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.detector = robot.intakeSensor;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;

            case Right:
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.CurveRight, 0.5, 30d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.3, 14d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.detector = robot.intakeSensor;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;
        }

        ThreadUtils.getExecutorService().submit(() -> {
            robot.outtake.lowerSlideForStonePickup();
            robot.intake.setPower(0);
            sleep(500);
            robot.outtake.outtakeClaw.setArmPosition(RobotStoneClaw.CLAW_OPEN_POSITION);
            sleep(500);
            robot.outtake.outtakeClaw.setArmPosition(RobotStoneClaw.CLAW_CLOSE_POSITION);
        });
    }

    private void crossSkyBridge(TargetPositionalDetector.Position position) {
        switch (position) {
            case Left:
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.5, 2d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.backCurving = true;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.CurveLeft, 0.5, 18d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.5, 30d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 13d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;
            case Center:
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 12d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.5, 50d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 15d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;
            case Right:
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 11d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.5, 60d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 15d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;
        }
    }

    private void moveFoundationToBuildingSite(TargetPositionalDetector.Position position) {
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.TurnRight, 0.3, Math.PI/2);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 9);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.foundationClaw.lowerClaw();

        // At the same time deploy stone to foundation to save time.
        ThreadUtils.getExecutorService().submit(() -> {
            robot.outtake.prepareForStoneDeployment();
            sleep(800);
            deploySkyStoneToFoundation();
        });

        sleep(800);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.5, 2);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.CurveLeft, 0.6, 30);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        gamepad.timeout = 1d;
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.5, 17);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        gamepad.timeout = 1d;
        gamepad.backCurving = true;
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.CurveRight, 0.5, 10);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.foundationClaw.raiseClaw();
    }

    private void deploySkyStoneToFoundation() {
        // lower slide for stone deployment
        ThreadUtils.getExecutorService().submit(() -> robot.outtake.outtakeSlides.runToPosition(20, true));
        sleep(500);

        robot.outtake.outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_OPEN_POSITION);

        sleep(500);

        robot.outtake.outtakeSlides.runToStoneHeight(robot.outtake.stoneHeight);

        sleep(500);

        ThreadUtils.getExecutorService().submit(() -> robot.outtake.performStoneIntakeOperation());
    }

    private void moveToParkUnderSkyBridge(TargetPositionalDetector.Position position) {
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 5d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(200);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.5, 35);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
    }
}
