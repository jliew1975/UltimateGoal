package org.firstinspires.ftc.teamcode.team12538.opModes.eBorg;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team12538.components.RobotStoneClaw;
import org.firstinspires.ftc.teamcode.team12538.detectors.TargetPositionalDetector;
import org.firstinspires.ftc.teamcode.team12538.detectors.TargetPositionalDetector.Position;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousColor;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousMode;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

import kotlin.Unit;

@Autonomous(name="Red (2 Stones And Foundation)", group="Group 1")
public class Red2StoneAndFoundationApp extends AutoLoadingZoneApp {
    public Red2StoneAndFoundationApp() {
        super();
        super.autoMode = AutonomousMode.RedLoading;
        super.autoColor = AutonomousColor.Red;
    }

    @Override
    protected void autoVisionLogic(TargetPositionalDetector detector) {
        Position skystonePosition = detector.getPosition();

        // set robot initial pose
        robot.drive.setPoseEstimate(new Pose2d(-35, -60, Math.toRadians(90)));

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
        }

        if(!isStopRequested()) {
            moveToParkUnderSkyBridge(position);
        }
    }

    private void pickupFirstStone(Position position) {
        // enable intake
        ThreadUtils.getExecutorService().submit(() -> robot.outtake.outtakeSlides.runToPosition(200));
        robot.intake.setPower(1);
        robot.intakeSensor.start();

        switch(position) {
            case Left:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder(SLOW_CONSTRAINTS)
                                .lineTo(new Vector2d(-32, -17.5), new ConstantInterpolator(Math.toRadians(165)))
                                .build()
                );

                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder(SLOW_CONSTRAINTS)
                                .forward(8)
                                .build()
                );

                break;

            case Center:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder(SLOW_CONSTRAINTS)
                                .lineTo(new Vector2d(-20, -17.5), new ConstantInterpolator(Math.toRadians(165)))
                                .build()
                );

                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder(SLOW_CONSTRAINTS)
                                .forward(8)
                                .build()
                );

                break;

            case Right:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder(SLOW_CONSTRAINTS)
                                .splineTo(new Pose2d(-32, -26, Math.toRadians(50)))
                                .forward(12)
                                .back(27)
                                .build()
                );
                break;
        }

        prepareStoneForDeployment();
    }

    private void pickupSecondStone(Position position) {
        ThreadUtils.getExecutorService().submit(() -> robot.outtake.outtakeSlides.runToPosition(200));
        robot.outtake.outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_INTAKE_POSITION);

        waitForStoneDeployment();

        switch (position) {
            case Left:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .splineTo(new Pose2d(0, -38, Math.toRadians(180)))
                                .lineTo(new Vector2d(-42, -36), new ConstantInterpolator(Math.toRadians(180)))
                                .lineTo(new Vector2d(-40, -16), new ConstantInterpolator(Math.toRadians(150)))
                                .build()
                );

                // enable intake
                robot.intake.setPower(1);

                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder(SLOW_CONSTRAINTS)
                                .forward(8)
                                .build()
                );

                break;

            case Center:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .splineTo(new Pose2d(0, -40, Math.toRadians(180)))
                                .lineTo(new Vector2d(-30, -40), new ConstantInterpolator(Math.toRadians(180)))
                                .lineTo(new Vector2d(-30, -10), new ConstantInterpolator(Math.toRadians(160)))
                                .build()
                );

                // enable intake
                robot.intake.setPower(1);

                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder(SLOW_CONSTRAINTS)
                                .forward(10)
                                .build()
                );

                break;

            case Right:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .splineTo(new Pose2d(0, -38, Math.toRadians(180)))
                                .lineTo(new Vector2d(-25, -38), new ConstantInterpolator(Math.toRadians(180)))
                                .lineTo(new Vector2d(-28, -12), new ConstantInterpolator(Math.toRadians(150)))
                                .build()
                );

                // enable intake
                robot.intake.setPower(1);

                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder(SLOW_CONSTRAINTS)
                                .forward(8)
                                .build()
                );

                break;
        }

        prepareStoneForDeployment();
    }

    private void crossSkyBridge(Position position, int round) {
        switch(position) {
            case Left:
                if(round == 1) {
                    robot.drive.followTrajectorySync(
                            robot.drive.trajectoryBuilder()
                                    .reverse()
                                    .lineTo(new Vector2d(0, -43), new SplineInterpolator(Math.toRadians(180), Math.toRadians(90)))
                                    .reverse()
                                    .lineTo(new Vector2d(45, -40), new SplineInterpolator(Math.toRadians(-180), Math.toRadians(-90)))
                                    .addMarker(new Vector2d(20, -40), () -> {
                                        ThreadUtils.getExecutorService().submit(() -> {
                                            deployStone(100);
                                        });
                                        return Unit.INSTANCE;
                                    })
                                    .build()
                    );
                } else {
                    robot.drive.followTrajectorySync(
                            robot.drive.trajectoryBuilder()
                                    .reverse()
                                    .splineTo(new Pose2d(0, -40, Math.toRadians(180)))
                                    .addMarker(new Vector2d(10, -38), () -> {
                                        ThreadUtils.getExecutorService().submit(() -> {
                                            deployStone(500, false);
                                        });
                                        return Unit.INSTANCE;
                                    })
                                    .reverse()
                                    .lineTo(new Vector2d(34, -50), new ConstantInterpolator(Math.toRadians(180)))
                                    .build()
                    );
                }
                break;
            case Center:
                if(round == 1) {
                    robot.drive.followTrajectorySync(
                            robot.drive.trajectoryBuilder()
                                    .reverse()
                                    .lineTo(new Vector2d(0, -45), new SplineInterpolator(Math.toRadians(180), Math.toRadians(90)))
                                    .reverse()
                                    .lineTo(new Vector2d(48, -40), new SplineInterpolator(Math.toRadians(-180), Math.toRadians(-90)))
                                    .addMarker(new Vector2d(20, -40), () -> {
                                        ThreadUtils.getExecutorService().submit(() -> {
                                            deployStone(100);
                                        });
                                        return Unit.INSTANCE;
                                    })
                                    .build()
                    );
                } else {
                    robot.drive.followTrajectorySync(
                            robot.drive.trajectoryBuilder()
                                    .reverse()
                                    .splineTo(new Pose2d(0, -40, Math.toRadians(180)))
                                    .addMarker(new Vector2d(0, -40), () -> {
                                        ThreadUtils.getExecutorService().submit(() -> {
                                            deployStone(500, false);
                                        });
                                        return Unit.INSTANCE;
                                    })
                                    .reverse()
                                    .lineTo(new Vector2d(34, -50), new ConstantInterpolator(Math.toRadians(180)))
                                    .build()
                    );
                }
                break;
            case Right:
                if(round == 1) {
                    robot.drive.followTrajectorySync(
                            robot.drive.trajectoryBuilder()
                                    .splineTo(new Pose2d(0, -40, 0))
                                    .addMarker(new Vector2d(20, -35), () -> {
                                        ThreadUtils.getExecutorService().submit(() -> {
                                            deployStone(100);
                                        });
                                        return Unit.INSTANCE;
                                    })
                                    .lineTo(new Vector2d(45, -35), new SplineInterpolator(Math.toRadians(0), Math.toRadians(-90)))
                                    .build()
                    );
                } else {
                    robot.drive.followTrajectorySync(
                            robot.drive.trajectoryBuilder()
                                    .reverse()
                                    .splineTo(new Pose2d(0, -40, Math.toRadians(180)))
                                    .addMarker(new Vector2d(10, -35), () -> {
                                        ThreadUtils.getExecutorService().submit(() -> {
                                            deployStone(500, false);
                                        });
                                        return Unit.INSTANCE;
                                    })
                                    .reverse()
                                    .lineTo(new Vector2d(34, -50), new ConstantInterpolator(Math.toRadians(180)))
                                    .build()
                    );
                }
                break;
        }

        if(round == 2 && robot.intake.isStuck()) {
            robot.drive.turnSync(Math.toRadians(-90));
            splitOutStone();
            robot.drive.turnSync(Math.toRadians(90));
        }
    }

    private void moveFoundationToBuildingSite(Position position) {
        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder(SLOW_CONSTRAINTS)
                        .back(10)
                        .build()
        );

        if(!robot.intakeSensor.isDetected()) {
            ThreadUtils.getExecutorService().submit(() -> splitOutStone());

        }

        robot.foundationClaw.lowerClaw();
        sleep(800);

        robot.drive.turnSync(Math.toRadians(-20));

        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .forward(25)
                        .build()
        );

        robot.drive.turnSync(Math.toRadians(-130));

        robot.foundationClaw.raiseClaw();
        sleep(200);
    }

    private void moveToParkUnderSkyBridge(Position position) {
        waitForStoneDeployment();

        double x = 11;
        double y = -40;

        if(position == Position.Left) {
            x = 10; y = -38;
        } else if(position == Position.Right) {
            x = 10; y = -38;
        }

        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .splineTo(new Pose2d(0, y, Math.toRadians(180)))
                        .build()
        );

        // robot.parkingServo.parkingMode();
    }
}
