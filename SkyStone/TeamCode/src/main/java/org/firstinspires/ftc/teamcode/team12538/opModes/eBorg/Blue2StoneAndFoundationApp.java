package org.firstinspires.ftc.teamcode.team12538.opModes.eBorg;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator;
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
                                .splineTo(new Pose2d(-30, 25, Math.toRadians(-45)))
                                .forward(5)
                                .back(25)
                                .build()
                );

                break;

            case Center:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .splineTo(new Pose2d(20, 10, Math.toRadians(-90)))
                                .strafeTo(new Vector2d(40, 10))
                                .forward(10d)
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
                robot.outtake.outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_OPEN_POSITION);
                sleep(200);
                robot.outtake.outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_CLOSE_POSITION);
            }
        });
    }

    private void pickupSecondStone(Position position) {
        ThreadUtils.getExecutorService().submit(() -> robot.outtake.outtakeSlides.runToPosition(200));
        robot.outtake.outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_INTAKE_POSITION);

        waitForStoneDeployment();

        switch (position) {
            case Left:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .splineTo(new Pose2d(0, 40, Math.toRadians(180)))
                                .build()
                );

                // enable intake
                robot.intake.setPower(1);

                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder(SLOW_CONSTRAINTS)
                                .lineTo(new Vector2d(-45, 25), new ConstantInterpolator(Math.toRadians(-160)))
                                .build()
                );

                break;

            case Center:

                break;

            case Right:

                break;
        }

        ThreadUtils.getExecutorService().submit(() -> {
            if(opModeIsActive()) {
                robot.outtake.lowerSlideForStonePickup();
                robot.intake.setPower(0);
                sleep(500);
                robot.outtake.outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_OPEN_POSITION);
                sleep(200);
                robot.outtake.outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_CLOSE_POSITION);
            }
        });
    }

    private void crossSkyBridge(Position position, int round) {
        switch(position) {
            case Left:
                if(round == 1) {
                    robot.drive.followTrajectorySync(
                            robot.drive.trajectoryBuilder()
                                    .splineTo(new Pose2d(0, 35, 0))
                                    .addMarker(new Vector2d(20, 35), () -> {
                                        ThreadUtils.getExecutorService().submit(() -> {
                                            if (robot.intakeSensor.isDetected()) {
                                                deployStone(10);
                                            }
                                        });
                                        return Unit.INSTANCE;
                                    })
                                    .lineTo(new Vector2d(45, 35), new SplineInterpolator(Math.toRadians(0), Math.toRadians(90)))
                                    .build()
                    );
                } else {
                    robot.drive.followTrajectorySync(
                            robot.drive.trajectoryBuilder()
                                    .reverse()
                                    .splineTo(new Pose2d(0, 35, Math.toRadians(180)))
                                    .build()
                    );

                    robot.drive.followTrajectorySync(
                            robot.drive.trajectoryBuilder()
                                    .addMarker(new Vector2d(20, 35), () -> {
                                        ThreadUtils.getExecutorService().submit(() -> {
                                            if (robot.intakeSensor.isDetected()) {
                                                deployStone(100);
                                            }
                                        });
                                        return Unit.INSTANCE;
                                    })
                                    .back(32)
                                    .build()
                    );
                }
                break;
            case Center:
                break;
            case Right:
                break;
        }
    }

    private void deploySecondStone(Position position) {
        if(opModeIsActive()) {
            // deployStone(100);
        }
    }

    private void moveFoundationToBuildingSite(Position position) {
        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .back(10)
                        .build()
        );

        robot.foundationClaw.lowerClaw();
        sleep(800);

        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .forward(15)
                        .build()
        );

        robot.drive.turnSync(Math.toRadians(120));

        robot.foundationClaw.raiseClaw();
        sleep(200);
    }

    private void moveToParkUnderSkyBridge(Position position) {
        waitForStoneDeployment();


        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .splineTo(new Pose2d(30, 35, Math.toRadians(-175)))
                        .build()
        );


        robot.parkingServo.parkingMode();
    }

    private void waitForStoneDeployment() {
        while(opModeIsActive() && !stoneDeployDone) {
            ThreadUtils.idle();
        }
    }
}
