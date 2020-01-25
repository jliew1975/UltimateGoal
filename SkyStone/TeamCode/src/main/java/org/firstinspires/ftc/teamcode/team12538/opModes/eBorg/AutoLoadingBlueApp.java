package org.firstinspires.ftc.teamcode.team12538.opModes.eBorg;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team12538.components.RobotStoneClaw;
import org.firstinspires.ftc.teamcode.team12538.detectors.TargetPositionalDetector;
import org.firstinspires.ftc.teamcode.team12538.detectors.TargetPositionalDetector.Position;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.utils.AutoGamepadUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousColor;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousMode;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

@Autonomous(name="Loading Blue", group="Linear Opmode")
public class AutoLoadingBlueApp extends AutoLoadingZoneApp {
    public AutoLoadingBlueApp() {
        super();
        super.autoMode = AutonomousMode.RedLoading;
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
            pickupFirstSkyStone(position);
        }

        if(opModeIsActive()) {
            crossSkyBridge(position, 1);
        }

        if(opModeIsActive()) {
            deployStone(20);
        }

        if(robot.intakeSensor.isDetected()) {
            if (opModeIsActive()) {
                pickupSecondSkyStone(position);
            }

            if (opModeIsActive()) {
                crossSkyBridge(position, 2);
            }

            if (opModeIsActive()) {
                if (position != Position.Left) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.3, 5);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }
                robot.outtake.stoneHeight = 2;
                deployStone(60);
            }
        }

        if(opModeIsActive()) {
            moveToParkUnderSkyBridge(position);
        }
    }

    private void pickupFirstSkyStone(Position position) {
        // lift slide for intake in a background thread
        ThreadUtils.getExecutorService().submit(() -> robot.outtake.outtakeSlides.runToPosition(50));

        // enable intake
        robot.intake.setPower(1);

        // start the distance sensor for stone detection
        robot.intakeSensor.start();

        while(robot.outtake.outtakeSlides.getCurrentPosition() > 60) {
            robot.outtake.outtakeSlides.runToPosition(50);
        }

        switch(position) {
            case Right:
                AutoGamepadUtils.move(gamepad, 0.5, 20d, -0.3, -0.31,false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.TurnRight, 0.5, Math.PI/2);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                robot.mecanumDrive.flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode.AudienceDirectionBlueAlliance);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 16,false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.detector = robot.intakeSensor;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 17d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;

            case Center:
                AutoGamepadUtils.move(gamepad, 0.5, 20d, -0.3, -0.5,false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.TurnRight, 0.5, Math.PI/2);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                robot.mecanumDrive.flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode.AudienceDirectionBlueAlliance);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 17,false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.detector = robot.intakeSensor;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 18d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;

            case Left:
                AutoGamepadUtils.move(gamepad, 0.8, 25d, -0.3, -0.23,false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.detector = robot.intakeSensor;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 12d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.5, 5d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.backCurving = true;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.CurveLeft, 0.5, 30d);
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
            robot.intake.setPower(-1);
            sleep(500);
            robot.intake.setPower(0);
        });
    }

    private void pickupSecondSkyStone(Position position) {
        // enable intake
        robot.intake.setPower(1);

        switch(position) {
            case Right:
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.8, 64);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 18);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.detector = robot.intakeSensor;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                ThreadUtils.getExecutorService().submit(() -> {
                    robot.outtake.lowerSlideForStonePickup();
                    robot.intake.setPower(0);
                    sleep(500);
                    robot.outtake.outtakeClaw.setArmPosition(RobotStoneClaw.CLAW_OPEN_POSITION);
                    sleep(500);
                    robot.outtake.outtakeClaw.setArmPosition(RobotStoneClaw.CLAW_CLOSE_POSITION);
                });
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 15d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;

            case Center:
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.8, 66);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 16);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.detector = robot.intakeSensor;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                ThreadUtils.getExecutorService().submit(() -> {
                    robot.outtake.lowerSlideForStonePickup();
                    robot.intake.setPower(0);
                    sleep(500);
                    robot.outtake.outtakeClaw.setArmPosition(RobotStoneClaw.CLAW_OPEN_POSITION);
                    sleep(500);
                    robot.outtake.outtakeClaw.setArmPosition(RobotStoneClaw.CLAW_CLOSE_POSITION);
                });
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 16d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;

            case Left:
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.8, 48);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 16);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.detector = robot.intakeSensor;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                ThreadUtils.getExecutorService().submit(() -> {
                    robot.outtake.lowerSlideForStonePickup();
                    robot.intake.setPower(0);
                    sleep(500);
                    robot.outtake.outtakeClaw.setArmPosition(RobotStoneClaw.CLAW_OPEN_POSITION);
                    sleep(500);
                    robot.outtake.outtakeClaw.setArmPosition(RobotStoneClaw.CLAW_CLOSE_POSITION);
                });
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 15d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;
        }
    }

    private void crossSkyBridge(Position position, int round) {
        double fastDistance = 0d;
        double slowDistance = 0d;

        robot.mecanumDrive.flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode.AudienceDirectionBlueAlliance);

        switch (position) {
            case Right:
                fastDistance = (round == 1) ? 50d : 75d;
                slowDistance = (round == 1) ? 5d : 5d;
                break;

            case Center:
                fastDistance = (round == 1) ? 48d : 72d;
                slowDistance = (round == 1) ? 3d : 3d;
                break;

            case Left:
                fastDistance = (round == 1) ? 40d : 65d;
                slowDistance = (round == 1) ? 3d : 3d;
                break;
        }

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.8, fastDistance, false);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, slowDistance, false);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
    }

    private void moveToParkUnderSkyBridge(Position position) {
        robot.mecanumDrive.flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode.AudienceDirectionBlueAlliance);
        if(position == Position.Left) {
            AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.6, 3, false);
            robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
        }
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.8, 15, false);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
    }
}
