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

@Autonomous(name="Blue (1 Stone And Foundation)", group="Group 2")
public class Blue1StoneAndFoundationApp extends AutoLoadingZoneApp {
    public Blue1StoneAndFoundationApp() {
        super();
        super.autoMode = AutonomousMode.BlueLoading;
        super.autoColor = AutonomousColor.Blue;
    }

    @Override
    protected void autoVisionLogic(TargetPositionalDetector detector) {
        TargetPositionalDetector.Position skystonePosition = detector.getPosition();

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

    protected void executeLogic(TargetPositionalDetector.Position position) {
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
                                .splineTo(new Pose2d(-29, 28, Math.toRadians(-45)))
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

    private void crossSkyBridge(Position position) {
        switch(position) {
            case Left:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .splineTo(new Pose2d(0, 38, 0))
                                .addMarker(new Vector2d(20, 35), () -> {
                                    ThreadUtils.getExecutorService().submit(() -> {
                                        deployStone(100);
                                    });
                                    return Unit.INSTANCE;
                                })
                                .lineTo(new Vector2d(50, 35), new SplineInterpolator(Math.toRadians(0), Math.toRadians(90)))
                                .build()
                );
                break;
            case Center:
                robot.drive.followTrajectorySync(
                        robot.drive.trajectoryBuilder()
                                .reverse()
                                .lineTo(new Vector2d(0, 45), new SplineInterpolator(Math.toRadians(-180), Math.toRadians(-90)))
                                .reverse()
                                .lineTo(new Vector2d(48, 40), new SplineInterpolator(Math.toRadians(180), Math.toRadians(90)))
                                .addMarker(new Vector2d(20, 35), () -> {
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
                                .reverse()
                                .lineTo(new Vector2d(0, 40), new SplineInterpolator(Math.toRadians(-180), Math.toRadians(-90)))
                                .reverse()
                                .lineTo(new Vector2d(48, 40), new SplineInterpolator(Math.toRadians(180), Math.toRadians(90)))
                                .addMarker(new Vector2d(20, 35), () -> {
                                    ThreadUtils.getExecutorService().submit(() -> {
                                        deployStone(100);
                                    });
                                    return Unit.INSTANCE;
                                })
                                .build()
                );
                break;
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

        robot.drive.turnSync(Math.toRadians(20));

        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .forward(22)
                        .build()
        );

        robot.drive.turnSync(Math.toRadians(120));

        robot.foundationClaw.raiseClaw();
        sleep(200);
    }

    private void moveToParkUnderSkyBridge(Position position) {
        waitForStoneDeployment();

        double y = 33;
        if(position == Position.Right || position == Position.Center) {
            y = 30;
        }

        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .splineTo(new Pose2d(8, y, Math.toRadians(180)))
                        .build()
        );

        // robot.parkingServo.parkingMode();
    }
}