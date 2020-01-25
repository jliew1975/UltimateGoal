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

@Autonomous(name="Loading Red", group="Linear Opmode")
public class AutoLoadingRedApp extends AutoLoadingZoneApp {
    public AutoLoadingRedApp() {
        super();
        super.autoMode = AutonomousMode.RedLoading;
        super.autoColor = AutonomousColor.Red;
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

        if(!robot.intakeSensor.isDetected()) {
            if (opModeIsActive()) {
                pickupSecondSkyStone(position);
            }

            if (opModeIsActive()) {
                crossSkyBridge(position, 2);
            }

            if (opModeIsActive()) {
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.3, 5);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
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

        switch(position) {
            case Right:
                AutoGamepadUtils.move(gamepad, 0.8, 15d, -0.3, 0.28,false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 16d,false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 10d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.TurnLeft, 0.5, Math.PI/2);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;
            case Center:
                AutoGamepadUtils.move(gamepad, 0.8, 20d, -0.3, 0.31,false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.TurnLeft, 0.5, Math.PI/2);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                robot.mecanumDrive.flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode.AudienceDirectionRedAlliance);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.6, 15, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.detector = robot.intakeSensor;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.6, 15, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;

            case Left:
                AutoGamepadUtils.move(gamepad, 0.8, 17d, -0.3, 0.24,false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.TurnLeft, 0.5, Math.PI/2);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                robot.mecanumDrive.flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode.AudienceDirectionRedAlliance);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.6, 15.5, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.detector = robot.intakeSensor;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.6, 14.5, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;
        }

        ThreadUtils.getExecutorService().submit(() -> {
            robot.outtake.lowerSlideForStonePickup();
            robot.intake.setPower(-1);
            sleep(500);
            robot.intake.setPower(0);
            robot.outtake.outtakeClaw.setArmPosition(RobotStoneClaw.CLAW_OPEN_POSITION);
            sleep(500);
            robot.outtake.outtakeClaw.setArmPosition(RobotStoneClaw.CLAW_CLOSE_POSITION);
        });
    }

    private void pickupSecondSkyStone(Position position) {
        // enable intake
        robot.intake.setPower(1);

        while(opModeIsActive() && robot.outtake.outtakeSlides.getCurrentPosition() > 60) {
            robot.outtake.outtakeSlides.runToPosition(50);
        }

        switch(position) {
            case Right:
                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.8, 50, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.6, 12.5, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    gamepad.detector = robot.intakeSensor;
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.6, 14d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }
                break;

            case Center:
                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.8, 58, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 12.5, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }
                if(opModeIsActive()) {
                    gamepad.detector = robot.intakeSensor;
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.6, 14d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }
                break;

            case Left:
                gamepad.timeout = 3;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.8, 62, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.timeout = 2;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.6, 13.5, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.detector = robot.intakeSensor;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 6d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                ThreadUtils.getExecutorService().submit(() -> robot.outtake.lowerSlideForStonePickup());
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.6, 14d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;
        }

        ThreadUtils.getExecutorService().submit(() -> {
            robot.outtake.lowerSlideForStonePickup();
            robot.intake.setPower(-1);
            sleep(500);
            robot.intake.setPower(0);
            robot.outtake.outtakeClaw.setArmPosition(RobotStoneClaw.CLAW_OPEN_POSITION);
            sleep(500);
            robot.outtake.outtakeClaw.setArmPosition(RobotStoneClaw.CLAW_CLOSE_POSITION);
        });
    }

    private void crossSkyBridge(Position position, int round) {
        double fastDistance = 0d;
        double slowDistance = 0d;

        robot.mecanumDrive.flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode.AudienceDirectionRedAlliance);

        switch (position) {
            case Right:
                fastDistance = (round == 1) ? 30d : 42d;
                slowDistance = (round == 1) ? 18d : 20d;
                break;

            case Center:
                fastDistance = (round == 1) ? 30d : 45d;
                slowDistance = (round == 1) ? 18d : 22d;
                break;

            case Left:
                fastDistance = (round == 1) ? 38d : 55d;
                slowDistance = (round == 1) ? 25d : 27d;
                break;
        }

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.8, fastDistance, false);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, slowDistance, false);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
    }

    private void moveToParkUnderSkyBridge(Position position) {
        if(position != Position.Right) {
            AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.6, 5d, false);
            robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
        }

        while(robot.outtake.outtakeSlides.getCurrentPosition() > 60) {
            robot.outtake.outtakeSlides.runToPosition(50);
        }

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.8, 15);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
    }
}
