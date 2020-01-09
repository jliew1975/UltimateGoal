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

    protected void executeLeftLogic() {
        if(opModeIsActive()) {
            pickupSkyStone(TargetPositionalDetector.Position.Left);
        }

        if(opModeIsActive()) {
            crossSkyBridge(TargetPositionalDetector.Position.Left);
        }

        if(opModeIsActive()) {
            moveFoundationToBuildingSite(TargetPositionalDetector.Position.Left);
        }

        if(opModeIsActive()) {
            moveToParkUnderSkyBridge(TargetPositionalDetector.Position.Left);
        }
    }

    protected void executeCenterLogic() {
        if(opModeIsActive()) {
            pickupSkyStone(TargetPositionalDetector.Position.Center);
        }

        if(opModeIsActive()) {
            crossSkyBridge(TargetPositionalDetector.Position.Center);
        }

        if(opModeIsActive()) {
            moveFoundationToBuildingSite(TargetPositionalDetector.Position.Center);
        }

        if(opModeIsActive()) {
            moveToParkUnderSkyBridge(TargetPositionalDetector.Position.Center);
        }
    }

    protected void executeRightLogic() {
        if(opModeIsActive()) {
            pickupSkyStone(TargetPositionalDetector.Position.Right);
        }

        if(opModeIsActive()) {
            crossSkyBridge(TargetPositionalDetector.Position.Right);
        }

        if(opModeIsActive()) {
            moveFoundationToBuildingSite(TargetPositionalDetector.Position.Right);
        }

        if(opModeIsActive()) {
            moveToParkUnderSkyBridge(TargetPositionalDetector.Position.Right);
        }
    }

    private void pickupSkyStone(TargetPositionalDetector.Position position) {
        // lift slide for intake in a background thread
        ThreadUtils.getExecutorService().submit(() -> robot.outtake.outtakeSlides.runToPosition(50));

        // enable intake
        robot.intake.setPower(1);

        // start the distance sensor for stone detection
        robot.intakeSensor.start();

        switch(position) {
            case Left:
                AutoGamepadUtils.move(gamepad, 0.8, 15d, -0.3, -0.2,false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.TurnRight, 0.5, Math.PI/2);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                robot.mecanumDrive.flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode.AudienceDirection);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.6, 15d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.detector = robot.intakeSensor;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.6, 10d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;

            case Center:
                AutoGamepadUtils.move(gamepad, 0.8, 15d, -0.3, -0.52 ,false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.TurnRight, 0.6, Math.PI/2);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                robot.mecanumDrive.flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode.AudienceDirection);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.6, 17d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.detector = robot.intakeSensor;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.6, 12d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;

            case Right:
                AutoGamepadUtils.move(gamepad, 0.8, 15d, -0.3, -0.35,false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.TurnRight, 0.3, Math.PI/2);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                robot.mecanumDrive.flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode.AudienceDirection);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.6, 18.5d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.detector = robot.intakeSensor;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.6, 12d, false);
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
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.8, 75d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;
            case Center:
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.8, 58d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;
            case Right:
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.8, 68d, false);
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
        robot.mecanumDrive.flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode.AudienceDirection);
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 7d, false);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(200);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.5, 35, false);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
    }
}
