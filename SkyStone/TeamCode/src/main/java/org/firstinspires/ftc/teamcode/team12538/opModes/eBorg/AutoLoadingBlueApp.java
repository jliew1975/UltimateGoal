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

@Autonomous(name="Blue (2 Stones)", group="Group 3")
public class AutoLoadingBlueApp extends AutoLoadingZoneApp {
    public AutoLoadingBlueApp() {
        super();
        super.autoMode = AutonomousMode.RedLoading;
        super.autoColor = AutonomousColor.Blue;
    }

    @Override
    protected void autoVisionLogic(TargetPositionalDetector detector) {
        Position skystonePosition = detector.getPosition();

        // set robot initial pose
        robot.drive.setPoseEstimate(new Pose2d(-38, 62, Math.toRadians(-90)));

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
            pickupFirstSkyStone(position);
        }

        if(!isStopRequested()) {
            crossSkyBridge(position, 1);
        }

        if(!robot.intakeSensor.isDetected()) {
            if (!isStopRequested()) {
                pickupSecondSkyStone(position);
            }

            if (!isStopRequested()) {
                crossSkyBridge(position, 2);
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
                                .splineTo(new Pose2d(-29, 25, Math.toRadians(-45)))
                                .forward(5)
                                .back(27)
                                .build()
                );

                break;

            case Center:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder(SLOW_CONSTRAINTS)
                                .lineTo(new Vector2d(-20, 18.5), new ConstantInterpolator(Math.toRadians(-170)))
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
                        robot.drive.trajectoryBuilder(SLOW_CONSTRAINTS)
                                .lineTo(new Vector2d(-31, 18), new ConstantInterpolator(Math.toRadians(-170)))
                                .build()
                );

                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder(SLOW_CONSTRAINTS)
                                .forward(5)
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
                                .splineTo(new Pose2d(0, 38, Math.toRadians(180)))
                                .lineTo(new Vector2d(-30, 35), new ConstantInterpolator(Math.toRadians(180)))
                                .lineTo(new Vector2d(-24, 16.5), new ConstantInterpolator(Math.toRadians(-150)))
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
                        robot.drive.trajectoryBuilder()
                                .splineTo(new Pose2d(0, 40, Math.toRadians(180)))
                                .lineTo(new Vector2d(-33, 40), new ConstantInterpolator(Math.toRadians(180)))
                                .lineTo(new Vector2d(-33, 16.5), new ConstantInterpolator(Math.toRadians(-160)))
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
                        robot.drive.trajectoryBuilder()
                                .splineTo(new Pose2d(0, 40, Math.toRadians(180)))
                                .lineTo(new Vector2d(-38, 48), new ConstantInterpolator(Math.toRadians(180)))
                                .lineTo(new Vector2d(-42, 16.5), new ConstantInterpolator(Math.toRadians(-165)))
                                .build()
                );

                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder(SLOW_CONSTRAINTS)
                                .forward(10)
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
                                    .splineTo(new Pose2d(0, 40, 0))
                                    .lineTo(new Vector2d(20, 40), new SplineInterpolator(Math.toRadians(0), Math.toRadians(90)))
                                    .build()
                    );

                    deployStone(200);
                } else {
                    robot.drive.followTrajectorySync(
                            robot.drive.trajectoryBuilder()
                                    .reverse()
                                    .splineTo(new Pose2d(0, 35, Math.toRadians(180)))
                                    .addMarker(new Vector2d(10, 35), () -> {
                                        ThreadUtils.getExecutorService().submit(() -> {
                                            deployStone(500);
                                        });
                                        return Unit.INSTANCE;
                                    })
                                    .reverse()
                                    .lineTo(new Vector2d(25, 33), new ConstantInterpolator(Math.toRadians(-165)))
                                    .build()
                    );
                }
                break;
            case Center:
                if(round == 1) {
                    robot.drive.followTrajectorySync(
                            robot.drive.trajectoryBuilder()
                                    .reverse()
                                    .lineTo(new Vector2d(0, 43), new SplineInterpolator(Math.toRadians(-180), Math.toRadians(-90)))
                                    .reverse()
                                    .lineTo(new Vector2d(25, 40), new SplineInterpolator(Math.toRadians(180), Math.toRadians(90)))
                                    .build()
                    );

                    deployStone(200);
                } else {
                    robot.drive.followTrajectorySync(
                            robot.drive.trajectoryBuilder()
                                    .reverse()
                                    .splineTo(new Pose2d(0, 38, Math.toRadians(180)))
                                    .addMarker(new Vector2d(10, 33), () -> {
                                        ThreadUtils.getExecutorService().submit(() -> {
                                            if (robot.intakeSensor.isDetected()) {
                                                deployStone(500);
                                            }
                                        });
                                        return Unit.INSTANCE;
                                    })
                                    .reverse()
                                    .lineTo(new Vector2d(25, 33), new ConstantInterpolator(Math.toRadians(-165)))
                                    .build()
                    );
                }
                break;
            case Right:
                if(round == 1) {
                    robot.drive.followTrajectorySync(
                            robot.drive.trajectoryBuilder()
                                    .reverse()
                                    .lineTo(new Vector2d(0, 40), new SplineInterpolator(Math.toRadians(-180), Math.toRadians(-90)))
                                    .reverse()
                                    .lineTo(new Vector2d(25, 40), new SplineInterpolator(Math.toRadians(180), Math.toRadians(90)))
                                    .build()
                    );
                    deployStone(200);
                } else {
                    robot.drive.followTrajectorySync(
                            robot.drive.trajectoryBuilder()
                                    .reverse()
                                    .splineTo(new Pose2d(0, 30, Math.toRadians(180)))
                                    .addMarker(new Vector2d(10, 28), () -> {
                                        ThreadUtils.getExecutorService().submit(() -> {
                                            if (robot.intakeSensor.isDetected()) {
                                                deployStone(500);
                                            }
                                        });
                                        return Unit.INSTANCE;
                                    })
                                    .reverse()
                                    .lineTo(new Vector2d(27, 28), new ConstantInterpolator(Math.toRadians(180)))
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

    private void moveToParkUnderSkyBridge(Position position) {
        waitForStoneDeployment();

        double y = 35;
        if(position == Position.Right) {
            y = 30;
        }

        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .splineTo(new Pose2d(0, y, Math.toRadians(180)))
                        .build()
        );

        // robot.parkingServo.parkingMode();
    }
}
