package org.firstinspires.ftc.teamcode.team12538.opModes.eBorg;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
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

@Autonomous(name="Red 2 Stone And Foundation", group="Linear Opmode")
public class Red2StoneAndFoundationApp extends AutoLoadingZoneApp {
    public Red2StoneAndFoundationApp() {
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

        sleep(500);
    }

    protected void executeLogic(Position position) {
        pickupFirstStone(position);

        crossSkyBridge(position, 1);

        /*
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
        */
    }

    private void pickupFirstStone(Position position) {
        // lift slide for intake in a background thread
        ThreadUtils.getExecutorService().submit(() -> robot.outtake.outtakeSlides.runToPosition(50));

        // enable intake
        robot.intake.setPower(1);

        // start the distance sensor for stone detection
        robot.intakeSensor.start();

        switch(position) {
            case Right:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .splineTo(new Pose2d(20, 30))
                                .forward(10)
                                .build()
                );

                break;
            case Center:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .splineTo(new Pose2d(20, 30, Math.toRadians(90)))
                                .strafeTo(new Vector2d(30, 10))
                                .build()
                );

                break;

            case Left:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .splineTo(new Pose2d(20, 10, Math.toRadians(90)))
                                .strafeTo(new Vector2d(30, 10))
                                .build()
                );

                break;
        }

        ThreadUtils.getExecutorService().submit(() -> {
            if(opModeIsActive()) {
                robot.outtake.lowerSlideForStonePickup();
                robot.intake.setPower(-1);
                sleep(500);
                robot.intake.setPower(0);
                robot.outtake.outtakeClaw.setArmPosition(RobotStoneClaw.CLAW_OPEN_POSITION);
                sleep(500);
                robot.outtake.outtakeClaw.setArmPosition(RobotStoneClaw.CLAW_CLOSE_POSITION);
            }
        });
    }

    private void pickupSecondStone(Position position) {

        robot.mecanumDrive.flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode.AudienceDirectionRedAlliance);

        // lift slide for intake in a background thread
        ThreadUtils.getExecutorService().submit(() -> robot.outtake.outtakeSlides.runToPosition(50));

        // enable intake
        robot.intake.setPower(1);

        while(opModeIsActive() && robot.outtake.outtakeSlides.getCurrentPosition() > 50) {
            robot.outtake.outtakeSlides.runToPosition(50);
        }

        switch (position) {
            case Right:
                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.8, 63d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.6, 14d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    gamepad.detector = robot.intakeSensor;
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 6d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.6, 14d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }
                break;
            case Center:
                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.8, 70d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.6, 14d, false);
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
            case Left:
                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.8, 78d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.6, 14.5d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    gamepad.detector = robot.intakeSensor;
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 6d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }

                if(opModeIsActive()) {
                    AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.6, 13d, false);
                    robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                }
                break;
        }

        ThreadUtils.getExecutorService().submit(() -> {
            if(opModeIsActive()) {
                robot.outtake.lowerSlideForStonePickup();
                robot.intake.setPower(-1);
                sleep(500);
                robot.intake.setPower(0);
                robot.outtake.outtakeClaw.setArmPosition(RobotStoneClaw.CLAW_OPEN_POSITION);
                sleep(500);
                robot.outtake.outtakeClaw.setArmPosition(RobotStoneClaw.CLAW_CLOSE_POSITION);
            }
        });

    }

    private void crossSkyBridge(Position position, int round) {
        double distanceToFoundation = 0d;

        robot.mecanumDrive.flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode.AudienceDirectionRedAlliance);

        switch (position) {
            case Right:
                distanceToFoundation = (round == 1) ? 56d : 78d;
                break;
            case Center:
                distanceToFoundation = (round == 1) ? 62d : 82d;
                break;
            case Left:
                // distanceToFoundation = (round == 1) ? 75d : 80d;
                distanceToFoundation = (round == 1) ? 68d : 92d;
                break;

        }

        if(opModeIsActive()) {
            AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.8, distanceToFoundation);
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
            AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.TurnLeft, 0.5, Math.PI / 2);
            robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
        }

        if(opModeIsActive()) {
            AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 8);
            robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
        }

        robot.foundationClaw.lowerClaw();

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
            AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.CurveRight, 0.8, 31);
            robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
        }

        if(opModeIsActive()) {
            gamepad.timeout = 1d;
            AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.4, 10);
            robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
        }

        double strafeDistance = 5;

        if(opModeIsActive()) {
            gamepad.timeout = 1d;
            AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.8, strafeDistance);
            robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
        }

        if(opModeIsActive()) {
            gamepad.timeout = 1d;
            gamepad.backCurving = true;
            AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.CurveLeft, 0.8, 10);
            robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
        }

        robot.foundationClaw.raiseClaw();

        // AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.6, 3, false);
        // robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
    }

    private void moveToParkUnderSkyBridge(Position position) {
        robot.mecanumDrive.flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode.AudienceDirectionRedAlliance);
        double parkDistance = 25d;

        while(opModeIsActive() && robot.outtake.outtakeSlides.getCurrentPosition() > 60) {
            robot.outtake.outtakeSlides.runToPosition(50);
        }

        if(opModeIsActive()) {
            AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.8, parkDistance, false);
            robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
        }
    }
}
