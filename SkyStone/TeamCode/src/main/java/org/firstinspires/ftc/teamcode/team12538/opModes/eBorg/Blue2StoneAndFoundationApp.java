package org.firstinspires.ftc.teamcode.team12538.opModes.eBorg;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team12538.components.RobotStoneClaw;
import org.firstinspires.ftc.teamcode.team12538.detectors.TargetPositionalDetector;
import org.firstinspires.ftc.teamcode.team12538.detectors.TargetPositionalDetector.Position;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.opModes.eBorg.AutoLoadingZoneApp;
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

        robot.mecanumDrive.stop();
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
                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, 0.8, 20d, -0.3, -0.25, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    gamepad.detector = robot.intakeSensor;
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 15d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.8, 8d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.TurnRight, 0.5, Math.PI / 2);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }
                break;

            case Center:
                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, 0.8, 15d, -0.3, -0.52, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.TurnRight, 0.5, Math.PI / 2);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    robot.mecanumDrive.flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode.AudienceDirectionBlueAlliance);
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.6, 17d, false);
                }

                if(opModeIsActive()) {
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                    gamepad.detector = robot.intakeSensor;
                }

                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.6, 12d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }
                break;

            case Right:
                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, 0.8, 15d, -0.3, -0.33, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.TurnRight, 0.5, Math.PI / 2);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    robot.mecanumDrive.flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode.AudienceDirectionBlueAlliance);
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.6, 16d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    gamepad.detector = robot.intakeSensor;
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.6, 12d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }
                break;
        }

        ThreadUtils.getExecutorService().submit(() -> {
            if(opModeIsActive()) {
                robot.outtake.lowerSlideForStonePickup();
                robot.intake.setPower(0);
                sleep(500);
                robot.outtake.outtakeClaw.setArmPosition(RobotStoneClaw.CLAW_OPEN_POSITION);
                sleep(500);
                robot.outtake.outtakeClaw.setArmPosition(RobotStoneClaw.CLAW_CLOSE_POSITION);
            }
        });
    }

    private void pickupSecondStone(Position position) {
        robot.mecanumDrive.flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode.AudienceDirectionBlueAlliance);

        // lift slide for intake in a background thread
        ThreadUtils.getExecutorService().submit(() -> robot.outtake.outtakeSlides.runToPosition(50));

        // enable intake
        robot.intake.setPower(1);

        while(opModeIsActive() && robot.outtake.outtakeSlides.getCurrentPosition() > 60) {
            // waiting for slide to come down
            robot.outtake.outtakeSlides.runToPosition(50);
        }

        switch (position) {
            case Left:
                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.6, 5d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.8, 62d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.6, 14d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    gamepad.timeout = 2;
                    gamepad.detector = robot.intakeSensor;
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 6d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.6, 13, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }
                break;

            case Center:
                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.6, 5d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.8, 68d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.6, 14.5d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    gamepad.timeout = 2;
                    gamepad.detector = robot.intakeSensor;
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.6, 14d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }
                break;

            case Right:
                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.6, 5d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.8, 75d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.6, 15d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    gamepad.timeout = 2;
                    gamepad.detector = robot.intakeSensor;
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.6, 13d);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }
                break;
        }

        ThreadUtils.getExecutorService().submit(() -> {
            if(opModeIsActive()) {
                robot.outtake.lowerSlideForStonePickup();
                robot.intake.setPower(0);
                sleep(500);
                robot.outtake.outtakeClaw.setArmPosition(RobotStoneClaw.CLAW_OPEN_POSITION);
                sleep(500);
                robot.outtake.outtakeClaw.setArmPosition(RobotStoneClaw.CLAW_CLOSE_POSITION);
            }
        });
    }

    private void crossSkyBridge(Position position, int round) {
        double distanceToFoundation = 0d;

        robot.mecanumDrive.flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode.AudienceDirectionBlueAlliance);

        switch (position) {
            case Left:
                distanceToFoundation = (round == 1) ? 62d : 83d;
                break;
            case Center:
                distanceToFoundation = (round == 1) ? 64d : 85d;
                break;
            case Right:
                distanceToFoundation = (round == 1) ? 66d : 88d;
                break;
        }

        if(opModeIsActive()) {
            AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.8, distanceToFoundation, false);
            robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
        }
    }

    private void deploySecondStone(Position position) {
        if(robot.intakeSensor.isDetected()) {
            // At the same time deploy stone
            if(opModeIsActive()) {
                deployStone(10);
            }
        }
    }

    private void moveFoundationToBuildingSite(Position position) {
        if(opModeIsActive()) {
            AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.TurnRight, 0.5, Math.PI / 2);
            robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
        }

        if(opModeIsActive()) {
            AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.2, 10);
            robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
        }

        if(opModeIsActive()) {
            robot.foundationClaw.lowerClaw();
        }

        // At the same time deploy stone to foundation to save time.
        if(robot.intakeSensor.isDetected()) {
            ThreadUtils.getExecutorService().submit(() -> {
                if(opModeIsActive()) {
                    deployStone(20);
                }
            });
        }

        sleep(800);

        if(opModeIsActive()) {
            AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.8, 2);
            robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
        }

        if(opModeIsActive()) {
            AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.CurveLeft, 0.8, 26);
            robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
        }

        if(opModeIsActive()) {
            gamepad.timeout = 1d;
            AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.4, 15);
            robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
        }

        if(opModeIsActive()) {
            robot.foundationClaw.raiseClaw();
        }
    }

    private void moveToParkUnderSkyBridge(Position position) {
        while(opModeIsActive() && robot.outtake.outtakeSlides.getCurrentPosition() > 60) {
            robot.outtake.outtakeSlides.runToStoneHeight(40);
        }

        if(opModeIsActive()) {
            AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.8, 25, false);
            robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
        }
    }
}
