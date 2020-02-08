package org.firstinspires.ftc.teamcode.team12538.opModes.eBorg;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.path.heading.TangentInterpolator;
import com.acmerobotics.roadrunner.path.heading.WiggleInterpolator;
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

import kotlin.Unit;

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
        pickupFirstStone(position);

        // crossSkyBridge(position, 1);

        /*
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
        */
    }

    private void pickupFirstStone(Position position) {
        // lift slide for intake in a background thread
        ThreadUtils.getExecutorService().submit(() -> robot.outtake.outtakeSlides.runToPosition(50));

        // enable intake
        robot.intake.setPower(1);

        switch(position) {
            case Left:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .splineTo(new Pose2d(40, 12, Math.toRadians(45)))
                                .forward(2)
                                .back(30)
                                .addMarker(() -> {
                                    robot.intake.setPower(0);
                                    return Unit.INSTANCE;
                                })
                                .splineTo(new Pose2d(25, 40, Math.toRadians(90)))
                                .lineTo(new Vector2d(25, 86), new SplineInterpolator(Math.toRadians(90), Math.toRadians(180)))
                                .build()
                );

                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .back(13)
                                .addMarker(() -> {
                                    robot.foundationClaw.lowerClaw();
                                    sleep(800);
                                    return Unit.INSTANCE;
                                })
                                .build()
                );



                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                            .forward(30)
                            .build()
                );

                robot.drive.turnSync(Math.toRadians(90));

                robot.foundationClaw.raiseClaw();
                sleep(800);

                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                            .splineTo(new Pose2d(1, 20, Math.toRadians(-90)))
                            .build()
                );

                // robot.intake.setPower(1);

                /*
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .splineTo(new Pose2d(1, -8, Math.toRadians(-90)))
                                .strafeLeft(18)
                                .build()
                );
                */
                /*
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .forward(8)
                                .build()
                );


                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .reverse()
                                .splineTo(new Pose2d(1, 30))
                                .build()
                );
                */

                break;

            case Center:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .splineTo(new Pose2d(20, 10, Math.toRadians(-90)))
                                .strafeTo(new Vector2d(40, 10))
                                .forward(5d)
                                .strafeTo(new Vector2d(30, 15))
                                .back(80)
                                .build()
                );
                break;

            case Right:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .splineTo(new Pose2d(20, 7))
                                .forward(10)
                                .build()
                );
                break;
        }

        sleep(500);

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
