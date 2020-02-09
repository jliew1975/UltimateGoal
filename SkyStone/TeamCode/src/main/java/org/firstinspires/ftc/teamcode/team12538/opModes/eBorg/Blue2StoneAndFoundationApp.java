package org.firstinspires.ftc.teamcode.team12538.opModes.eBorg;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.path.heading.TangentInterpolator;
import com.acmerobotics.roadrunner.path.heading.WiggleInterpolator;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team12538.components.RobotStoneAligner;
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
    public static DriveConstraints SLOW_CONSTRAINTS = new DriveConstraints(
            30.0, 30.0, 0.0,
            Math.toRadians(180.0), Math.toRadians(180.0), 0.0
    );

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
        if(!isStopRequested()) {
            pickupFirstStone(position);
        }

        if(!isStopRequested()) {
            crossSkyBridge(position, 1);
        }

        if(!isStopRequested()) {
            moveFoundationToBuildingSite(position);
        }

        if(!robot.intakeSensor.isDetected()) {
            if (!isStopRequested()) {
                pickupSecondStone(position);
            }

            if (!isStopRequested()) {
                crossSkyBridge(position, 2);
            }

            if (!isStopRequested()) {
                deploySecondStone(position);
            }
        } else {
            if(!isStopRequested()) {
                deployStone(20);
            }
        }

        if(opModeIsActive()) {
            moveToParkUnderSkyBridge(position);
        }
    }

    private void pickupFirstStone(Position position) {
        // enable intake
        robot.intake.setPower(1);
        robot.intakeSensor.start();

        switch(position) {
            case Left:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder(SLOW_CONSTRAINTS)
                                .splineTo(new Pose2d(35, 5, Math.toRadians(30)))
                                .forward(10)
                                .back(30)
                                .build()
                );

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
        while(robot.outtake.outtakeSlides.getCurrentPosition() > 60) {
            robot.outtake.liftSlideForStoneIntake();
        }

        switch (position) {
            case Left:
                Pose2d error = robot.drive.getLastError();
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .splineTo(new Pose2d(20 + error.getX(), 30 + error.getY(), Math.toRadians(-90)))
                                .build()
                );

                // enable intake
                robot.intake.setPower(1);

                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .lineTo(new Vector2d(20, 20))
                                .strafeTo(new Vector2d(48, 20))
                                .forward(5)
                                .strafeTo(new Vector2d(25, 20))
                                .build()
                );

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
        switch(position) {
            case Left:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .splineTo(new Pose2d(28, 40, Math.toRadians(90)))
                                .build()
                );

                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .lineTo(new Vector2d(28, 86))
                                .build()
                );
                break;
            case Center:
                break;
            case Right:

        }
    }

    private void deploySecondStone(Position position) {
        if(robot.intakeSensor.isDetected() && opModeIsActive()) {
            deployStone(20);
        }
    }

    private void moveFoundationToBuildingSite(Position position) {
        robot.drive.turnSync(Math.PI/2);

        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .back(10)
                        .build()
        );

        if(robot.intakeSensor.isDetected()) {
            ThreadUtils.getExecutorService().submit(() -> deployStone(10));
        }

        robot.foundationClaw.lowerClaw();
        sleep(800);

        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .forward(30)
                        .build()
        );

        robot.drive.turnSync(Math.toRadians(90));

        robot.foundationClaw.raiseClaw();
        sleep(500);
    }

    private void moveToParkUnderSkyBridge(Position position) {
        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .splineTo(new Pose2d(30, 30, Math.toRadians(90)))
                        .build()
        );
    }
}
