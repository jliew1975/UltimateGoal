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

@Autonomous(name="Red (1 Stone And Foundation)", group="Group 2")
public class Red1StoneAndFoundationApp extends AutoLoadingZoneApp {
    public Red1StoneAndFoundationApp() {
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
    }

    protected void executeLogic(Position position) {
        if(!isStopRequested()) {
            pickupFirstStone(position);
        }

        if(!isStopRequested()) {
            crossSkyBridge(position);
        }

        if(!isStopRequested()) {
            moveFoundationToBuildingSite(position);
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
                                .splineTo(new Pose2d(-34, -27, Math.toRadians(50)))
                                .forward(10)
                                .back(27)
                                .build()
                );
                break;
        }

        prepareStoneForDeployment();
    }

    private void crossSkyBridge(Position position) {
        switch(position) {
            case Left:
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
                break;

            case Center:
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

                break;
            case Right:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .splineTo(new Pose2d(0, -38, 0))
                                .addMarker(new Vector2d(20, -35), () -> {
                                    ThreadUtils.getExecutorService().submit(() -> {
                                        deployStone(100);
                                    });
                                    return Unit.INSTANCE;
                                })
                                .lineTo(new Vector2d(45, -35), new SplineInterpolator(Math.toRadians(0), Math.toRadians(-90)))
                                .build()
                );

                break;
        }
    }

    private void moveFoundationToBuildingSite(Position position) {
        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
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

        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                    .back(10)
                    .build()
        );
    }

    private void moveToParkUnderSkyBridge(Position position) {
        waitForStoneDeployment();

        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .splineTo(new Pose2d(0, -38, Math.toRadians(180)))
                        .build()
        );
    }
}
