package org.firstinspires.ftc.teamcode.team12538;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team12538.components.RobotStoneClaw;
import org.firstinspires.ftc.teamcode.team12538.detectors.TargetPositionalDetector;
import org.firstinspires.ftc.teamcode.team12538.detectors.TargetPositionalDetector.Position;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.utils.AutoGamepadUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousColor;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousMode;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

@Autonomous(name="Blue 2 Stones And Foundation", group="Linear Opmode")
public class Blue2StoneAndFoundationApp extends AutoLoadingZoneApp {
    public Blue2StoneAndFoundationApp() {
        super();
        super.autoMode = AutonomousMode.BlueLoading;
        super.autoColor = AutonomousColor.Blue;
    }

    @Override
    protected void autoVisionLogic(TargetPositionalDetector detector) {
        Position skystonePosition = detector.getPosition();

        switch (skystonePosition) {
            case Left:
                executeLogic(Position.Left);
                break;
            case Center:
                executeLogic(Position.Center);
                break;
            default:
                executeLogic(Position.Right);
        }
    }

    protected void executeLogic(Position position) {
        if(opModeIsActive()) {
            pickupFirstStone(position);
        }

        if(opModeIsActive()) {
            crossSkyBridge(position, 1);
        }

        if(opModeIsActive()) {
            moveFoundationToBuildingSite(position);
        }

        if(!robot.intakeSensor.isDetected()) {
            if (opModeIsActive()) {
                pickupSecondStone(position);
            }

            if (opModeIsActive()) {
                crossSkyBridge(position, 2);
            }

            if (opModeIsActive()) {
                deploySecondStone(position);
            }
        }

        if(opModeIsActive()) {
            moveToParkUnderSkyBridge(position);
        }
    }

    private void pickupFirstStone(Position position) {
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
                robot.mecanumDrive.flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode.AudienceDirectionBlueAlliance);
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
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.TurnRight, 0.5, Math.PI/2);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                robot.mecanumDrive.flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode.AudienceDirectionBlueAlliance);
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
                robot.mecanumDrive.flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode.AudienceDirectionBlueAlliance);
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

    private void pickupSecondStone(Position position) {
        robot.mecanumDrive.flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode.AudienceDirectionRedAlliance);

        // lift slide for intake in a background thread
        ThreadUtils.getExecutorService().submit(() -> robot.outtake.outtakeSlides.runToPosition(50));

        // enable intake
        robot.intake.setPower(1);

        // start the distance sensor for stone detection
        robot.intakeSensor.start();

        switch (position) {
            case Left:
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.8, 73d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.6, 14.5d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.timeout = 2;
                gamepad.detector = robot.intakeSensor;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 6d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.6, 13, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;

            case Center:
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.8, 70d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.6, 14d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.timeout = 2;
                gamepad.detector = robot.intakeSensor;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.6, 14d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;

            case Right:
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.8, 63d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.6, 13d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.timeout = 2;
                gamepad.detector = robot.intakeSensor;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 6d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.6, 13, false);
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

    private void crossSkyBridge(Position position, int round) {
        double distanceToFoundation = 0d;

        robot.mecanumDrive.flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode.AudienceDirectionBlueAlliance);

        switch (position) {
            case Left:
                distanceToFoundation = (round == 1) ? 75d : 85d;
                break;
            case Center:
                distanceToFoundation = (round == 1) ? 58d : 68d;
                break;
            case Right:
                distanceToFoundation = (round == 1) ? 75d : 85d;
                break;
        }

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.8, 75d, false);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
    }

    private void deploySecondStone(Position position) {
        if(robot.intakeSensor.isDetected()) {
            // At the same time deploy stone
            deployStone(10);
        }
    }

    private void moveFoundationToBuildingSite(Position position) {
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.TurnRight, 0.5, Math.PI/2);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 8);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.foundationClaw.lowerClaw();

        sleep(800);

        // At the same time deploy stone to foundation to save time.
        if(robot.intakeSensor.isDetected()) {
            ThreadUtils.getExecutorService().submit(() -> {
                deployStone(20);
            });
        }

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.8, 2);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.CurveLeft, 0.8, 31);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        gamepad.timeout = 1d;
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.8, 10);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        gamepad.timeout = 1d;
        gamepad.backCurving = true;
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.CurveRight, 0.8, 10);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.foundationClaw.raiseClaw();
    }

    private void moveToParkUnderSkyBridge(Position position) {
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.8, 25, false);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
    }
}
