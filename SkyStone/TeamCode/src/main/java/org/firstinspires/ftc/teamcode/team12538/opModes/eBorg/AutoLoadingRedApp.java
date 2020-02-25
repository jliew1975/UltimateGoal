package org.firstinspires.ftc.teamcode.team12538.opModes.eBorg;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team12538.components.RobotStoneClaw;
import org.firstinspires.ftc.teamcode.team12538.detectors.TargetPositionalDetector;
import org.firstinspires.ftc.teamcode.team12538.detectors.TargetPositionalDetector.Position;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.utils.AutoGamepadUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousColor;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousMode;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

import kotlin.Unit;

@Autonomous(name="Red (2 Stones)", group="Group 3")
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
            deployStone(100);
        }

        if(!robot.intakeSensor.isDetected()) {
            if (opModeIsActive()) {
                pickupSecondSkyStone(position);
            }

            if (opModeIsActive()) {
                crossSkyBridge(position, 2);
            }

            if (opModeIsActive()) {
                deployStone(100);
            }
        }

        if(opModeIsActive()) {
            moveToParkUnderSkyBridge(position);
        }
    }

    private void pickupFirstSkyStone(Position position) {
        // enable intake
        ThreadUtils.getExecutorService().submit(() -> robot.outtake.outtakeSlides.runToPosition(200));
        robot.intake.setPower(1);
        robot.intakeSensor.start();

        switch(position) {
            case Left:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder(SLOW_CONSTRAINTS)
                                .lineTo(new Vector2d(-31, -17.5), new ConstantInterpolator(Math.toRadians(165)))
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
                                .splineTo(new Pose2d(-33, -28, Math.toRadians(45)))
                                .forward(5)
                                .back(27)
                                .build()
                );
                break;
        }

        prepareStoneForDeployment();
    }

    private void pickupSecondSkyStone(Position position) {
        ThreadUtils.getExecutorService().submit(() -> robot.outtake.outtakeSlides.runToPosition(200));
        robot.outtake.outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_INTAKE_POSITION);

        waitForStoneDeployment();

        // enable intake
        robot.intake.setPower(1);

        switch (position) {
            case Left:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .splineTo(new Pose2d(0, -38, Math.toRadians(180)))
                                .lineTo(new Vector2d(-40, -38), new ConstantInterpolator(Math.toRadians(180)))
                                .lineTo(new Vector2d(-48, -10), new ConstantInterpolator(Math.toRadians(160)))
                                .build()
                );

                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder(SLOW_CONSTRAINTS)
                                .forward(5)
                                .build()
                );

                break;

            case Center:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .splineTo(new Pose2d(0, -35, Math.toRadians(180)))
                                .lineTo(new Vector2d(-35, -35), new ConstantInterpolator(Math.toRadians(180)))
                                .lineTo(new Vector2d(-38, -10), new ConstantInterpolator(Math.toRadians(160)))
                                .build()
                );

                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder(SLOW_CONSTRAINTS)
                                .forward(5)
                                .build()
                );
                break;

            case Right:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .splineTo(new Pose2d(0, -38, Math.toRadians(180)))
                                .lineTo(new Vector2d(-25, -38), new ConstantInterpolator(Math.toRadians(180)))
                                .lineTo(new Vector2d(-28, -10), new ConstantInterpolator(Math.toRadians(150)))
                                .build()
                );

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
                                    .splineTo(new Pose2d(0, -35, 0))
                                    .addMarker(new Vector2d(10, -35), () -> {
                                        ThreadUtils.getExecutorService().submit(() -> {
                                            deployStone(100);
                                        });
                                        return Unit.INSTANCE;
                                    })
                                    .lineTo(new Vector2d(20, -35), new SplineInterpolator(Math.toRadians(0), Math.toRadians(-90)))
                                    .build()
                    );
                } else {
                    robot.drive.followTrajectorySync(
                            robot.drive.trajectoryBuilder()
                                    .reverse()
                                    .splineTo(new Pose2d(0, -35, Math.toRadians(180)))
                                    .addMarker(new Vector2d(10, -35), () -> {
                                        ThreadUtils.getExecutorService().submit(() -> {
                                            deployStone(100);
                                        });
                                        return Unit.INSTANCE;
                                    })
                                    .reverse()
                                    .lineTo(new Vector2d(32, -38), new ConstantInterpolator(Math.toRadians(165)))
                                    .build()
                    );
                }
                break;
            case Center:
                if(round == 1) {
                    robot.drive.followTrajectorySync(
                            robot.drive.trajectoryBuilder()
                                    .reverse()
                                    .lineTo(new Vector2d(0, -43), new SplineInterpolator(Math.toRadians(180), Math.toRadians(90)))
                                    .reverse()
                                    .lineTo(new Vector2d(20, -40), new SplineInterpolator(Math.toRadians(180), Math.toRadians(-90)))
                                    .addMarker(new Vector2d(10, -40), () -> {
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
                                    .splineTo(new Pose2d(0, -38, Math.toRadians(180)))
                                    .addMarker(new Vector2d(20, -38), () -> {
                                        ThreadUtils.getExecutorService().submit(() -> {
                                            deployStone(100);
                                        });
                                        return Unit.INSTANCE;
                                    })
                                    .reverse()
                                    .lineTo(new Vector2d(32, -38), new ConstantInterpolator(Math.toRadians(165)))
                                    .build()
                    );
                }
                break;
            case Right:
                if(round == 1) {
                    robot.drive.followTrajectorySync(
                            robot.drive.trajectoryBuilder()
                                    .reverse()
                                    .lineTo(new Vector2d(0, -40), new SplineInterpolator(Math.toRadians(180), Math.toRadians(90)))
                                    .lineTo(new Vector2d(20, -40), new SplineInterpolator(Math.toRadians(90), Math.toRadians(180)))
                                    .addMarker(new Vector2d(10, -40), () -> {
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
                                    .splineTo(new Pose2d(0, -30, Math.toRadians(180)))
                                    .addMarker(new Vector2d(10, -38), () -> {
                                        ThreadUtils.getExecutorService().submit(() -> {
                                            deployStone(100);
                                        });
                                        return Unit.INSTANCE;
                                    })
                                    .reverse()
                                    .lineTo(new Vector2d(32, -38), new ConstantInterpolator(Math.toRadians(165)))
                                    .build()
                    );
                }
                break;
        }
    }

    private void moveToParkUnderSkyBridge(Position position) {
        waitForStoneDeployment();

        double x = 11;
        double y = 35;

        if(position == Position.Left) {
            x = 10; y = 35;
        } else if(position == Position.Right) {
            x = 10; y = 33;
        }

        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .splineTo(new Pose2d(11, y, Math.toRadians(180)))
                        .build()
        );

        // robot.parkingServo.parkingMode();
    }
}
