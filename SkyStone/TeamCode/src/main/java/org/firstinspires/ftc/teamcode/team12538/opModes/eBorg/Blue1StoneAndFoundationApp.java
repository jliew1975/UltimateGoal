package org.firstinspires.ftc.teamcode.team12538.opModes.eBorg;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team12538.components.RobotStoneClaw;
import org.firstinspires.ftc.teamcode.team12538.detectors.TargetPositionalDetector;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.opModes.eBorg.AutoLoadingZoneApp;
import org.firstinspires.ftc.teamcode.team12538.utils.AutoGamepadUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousColor;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousMode;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

@Autonomous(name="Blue 1 Stone And Foundation", group="Linear Opmode")
public class Blue1StoneAndFoundationApp extends AutoLoadingZoneApp {
    public Blue1StoneAndFoundationApp() {
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

        while(robot.outtake.outtakeSlides.getCurrentPosition() > 60) {
            robot.outtake.outtakeSlides.runToPosition(50);
        }

        switch(position) {
            case Left:
                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, 0.8, 15d, -0.3, -0.2, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.TurnRight, 0.5, Math.PI / 2);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    robot.mecanumDrive.flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode.AudienceDirectionBlueAlliance);
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.6, 15d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    gamepad.detector = robot.intakeSensor;
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.6, 10d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }
                break;

            case Center:
                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, 0.8, 15d, -0.3, -0.52, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }
                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.TurnRight, 0.6, Math.PI / 2);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    robot.mecanumDrive.flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode.AudienceDirectionBlueAlliance);
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.6, 17d, false);
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

            case Right:
                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, 0.8, 15d, -0.3, -0.35, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.TurnRight, 0.3, Math.PI / 2);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    robot.mecanumDrive.flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode.AudienceDirectionBlueAlliance);
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.6, 18.5d, false);
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
            robot.outtake.lowerSlideForStonePickup();
            robot.intake.setPower(-1);
            sleep(500);
            robot.intake.setPower(0);
            robot.outtake.outtakeClaw.setArmPosition(RobotStoneClaw.CLAW_OPEN_POSITION);
            sleep(500);
            robot.outtake.outtakeClaw.setArmPosition(RobotStoneClaw.CLAW_CLOSE_POSITION);
        });
    }

    private void crossSkyBridge(TargetPositionalDetector.Position position) {
        double distanceToFoundation = 0d;

        robot.mecanumDrive.flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode.AudienceDirectionBlueAlliance);

        switch (position) {
            case Left:
                distanceToFoundation = 75d;
                break;
            case Center:
                distanceToFoundation = 58d;
                break;
            case Right:
                distanceToFoundation = 68d;
                break;
        }

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.8, distanceToFoundation, false);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
    }

    private void moveFoundationToBuildingSite(TargetPositionalDetector.Position position) {
        if(opModeIsActive()) {
            AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.TurnRight, 0.3, Math.PI / 2);
            robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
        }

        if(opModeIsActive()) {
            AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 9);
            robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
        }

        robot.foundationClaw.lowerClaw();

        // At the same time deploy stone to foundation to save time.
        ThreadUtils.getExecutorService().submit(() -> {
            if(opModeIsActive()) {
                deployStone(20);
            }
        });

        sleep(800);

        if(opModeIsActive()) {
            AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.5, 2);
            robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
        }

        if(opModeIsActive()) {
            AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.CurveLeft, 0.6, 30);
            robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
        }

        if(opModeIsActive()) {
            gamepad.timeout = 1d;
            AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.5, 17);
            robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
        }

        if(opModeIsActive()) {
            gamepad.timeout = 1d;
            gamepad.backCurving = true;
            AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.CurveRight, 0.5, 10);
            robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
        }

        robot.foundationClaw.raiseClaw();
    }

    private void moveToParkUnderSkyBridge(TargetPositionalDetector.Position position) {
        if(opModeIsActive()) {
            robot.mecanumDrive.flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode.AudienceDirectionBlueAlliance);
            AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 5d, false);
            robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
        }

        while(opModeIsActive() && robot.outtake.outtakeSlides.getCurrentPosition() > 60) {
            robot.outtake.outtakeSlides.runToPosition(50);
        }

        if(opModeIsActive()) {
            AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.5, 25, false);
            robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
        }
    }
}
