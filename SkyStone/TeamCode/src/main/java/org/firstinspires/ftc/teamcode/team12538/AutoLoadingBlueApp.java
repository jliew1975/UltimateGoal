package org.firstinspires.ftc.teamcode.team12538;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team12538.components.RobotStoneClaw;
import org.firstinspires.ftc.teamcode.team12538.detectors.TargetPositionalDetector;
import org.firstinspires.ftc.teamcode.team12538.detectors.TargetPositionalDetector.Position;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.utils.AutoGamepadUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousColor;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousMode;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

@Autonomous(name="Loading Blue", group="Linear Opmode")
public class AutoLoadingBlueApp extends AutoLoadingZoneApp {
    public AutoLoadingBlueApp() {
        super();
        super.autoMode = AutonomousMode.RedLoading;
        super.autoColor = AutonomousColor.Blue;
        // super.numSkystone = 2;
    }

    @Override
    protected void autoVisionLogic(TargetPositionalDetector detector) {
        Position skystonePosition = detector.getPosition();

        switch (skystonePosition) {
            case Left:
                executeLeftLogic(Position.Left);
                break;
            case Center:
                executeCenterLogic(Position.Center);
                break;
            default:
                executeRightLogic(Position.Right);
        }

        sleep(500);
    }

    protected void executeRightLogic(Position position) {
        if(opModeIsActive()) {
            pickupFirstSkyStone(position);
        }

        if(opModeIsActive()) {
            crossSkyBridge(position, 1);
        }

        if(opModeIsActive()) {
            deploySkyStoneToFoundation(20);
        }

        if(opModeIsActive()) {
            pickupSecondSkyStone(position);
        }

        if(opModeIsActive()) {
            crossSkyBridge(position, 2);
        }

        if(opModeIsActive()) {
            robot.outtake.stoneHeight = 2;
            deploySkyStoneToFoundation(60);
        }

        if(opModeIsActive()) {
            moveToParkUnderSkyBridge(position);
        }
    }

    protected void executeCenterLogic(Position position) {
        if(opModeIsActive()) {
            pickupFirstSkyStone(position);
        }

        if(opModeIsActive()) {
            crossSkyBridge(position, 1);
        }

        if(opModeIsActive()) {
            deploySkyStoneToFoundation(20);
        }

        if(opModeIsActive()) {
            pickupSecondSkyStone(position);
        }

        if(opModeIsActive()) {
            crossSkyBridge(position, 2);
        }

        if(opModeIsActive()) {
            robot.outtake.stoneHeight = 2;
            deploySkyStoneToFoundation(60);
        }

        if(opModeIsActive()) {
            moveToParkUnderSkyBridge(position);
        }
    }

    protected void executeLeftLogic(Position position) {
        if(opModeIsActive()) {
            pickupFirstSkyStone(position);
        }

        if(opModeIsActive()) {
            crossSkyBridge(position, 1);
        }

        if(opModeIsActive()) {
            deploySkyStoneToFoundation(20);
        }

        if(opModeIsActive()) {
            pickupSecondSkyStone(position);
        }

        if(opModeIsActive()) {
            crossSkyBridge(position, 2);
        }

        if(opModeIsActive()) {
            if(position != Position.Left) {
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.3, 5);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
            }
            robot.outtake.stoneHeight = 2;
            deploySkyStoneToFoundation(60);
        }

        if(opModeIsActive()) {
            moveToParkUnderSkyBridge(position);
        }
    }

    private void pickupFirstSkyStone(Position position) {
        // lift slide for intake in a background thread
        ThreadUtils.getExecutorService().submit(() -> robot.outtake.outtakeSlides.runToPosition(50));

        // enable intake
        robot.intake.setPower(1);

        // start the distance sensor for stone detection
        robot.intakeSensor.start();

        switch(position) {
            case Right:
                AutoGamepadUtils.move(gamepad, 0.5, 20d, -0.3, -0.31,false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.TurnRight, 0.5, Math.PI/2);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                robot.mecanumDrive.flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode.AudienceDirection);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 16,false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.detector = robot.intakeSensor;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 17d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;

            case Center:
                AutoGamepadUtils.move(gamepad, 0.5, 20d, -0.3, -0.5,false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.TurnRight, 0.5, Math.PI/2);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                robot.mecanumDrive.flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode.AudienceDirection);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 17,false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.detector = robot.intakeSensor;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 18d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;

            case Left:
                AutoGamepadUtils.move(gamepad, 0.8, 25d, -0.3, -0.23,false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.detector = robot.intakeSensor;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 12d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.5, 5d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.backCurving = true;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.CurveLeft, 0.5, 30d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;
        }

        ThreadUtils.getExecutorService().submit(() -> {
            robot.outtake.lowerSlideForStonePickup();
            robot.intake.setPower(0);
            sleep(500);
            robot.outtake.outtakeClaw.setArmPosition(RobotStoneClaw.CLAW_OPEN_POSITION);
            sleep(500);
            robot.outtake.outtakeClaw.setArmPosition(RobotStoneClaw.CLAW_CLOSE_POSITION);
            robot.intake.setPower(-1);
            sleep(500);
            robot.intake.setPower(0);
        });
    }

    private void pickupSecondSkyStone(Position position) {
        // enable intake
        robot.intake.setPower(1);

        switch(position) {
            case Right:
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.8, 64);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 18);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.detector = robot.intakeSensor;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                ThreadUtils.getExecutorService().submit(() -> {
                    robot.outtake.lowerSlideForStonePickup();
                    robot.intake.setPower(0);
                    sleep(500);
                    robot.outtake.outtakeClaw.setArmPosition(RobotStoneClaw.CLAW_OPEN_POSITION);
                    sleep(500);
                    robot.outtake.outtakeClaw.setArmPosition(RobotStoneClaw.CLAW_CLOSE_POSITION);
                });
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 15d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;

            case Center:
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.8, 66);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 16);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.detector = robot.intakeSensor;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                ThreadUtils.getExecutorService().submit(() -> {
                    robot.outtake.lowerSlideForStonePickup();
                    robot.intake.setPower(0);
                    sleep(500);
                    robot.outtake.outtakeClaw.setArmPosition(RobotStoneClaw.CLAW_OPEN_POSITION);
                    sleep(500);
                    robot.outtake.outtakeClaw.setArmPosition(RobotStoneClaw.CLAW_CLOSE_POSITION);
                });
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 16d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;

            case Left:
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.8, 48);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 16);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.detector = robot.intakeSensor;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                ThreadUtils.getExecutorService().submit(() -> {
                    robot.outtake.lowerSlideForStonePickup();
                    robot.intake.setPower(0);
                    sleep(500);
                    robot.outtake.outtakeClaw.setArmPosition(RobotStoneClaw.CLAW_OPEN_POSITION);
                    sleep(500);
                    robot.outtake.outtakeClaw.setArmPosition(RobotStoneClaw.CLAW_CLOSE_POSITION);
                });
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 15d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;
        }
    }

    private void crossSkyBridge(Position position, int round) {
        robot.mecanumDrive.flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode.AudienceDirection);

        switch (position) {
            case Right:
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.8, round == 1 ? 50d : 75d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 5d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;

            case Center:
                gamepad.stopMotor = false;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.6, round == 1 ? 48d : 72d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.stopMotor = true;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, round == 1 ? 3d : 3d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;

            case Left:
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.8, round == 1 ? 39d : 65d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;
        }
    }

    private void deploySkyStoneToFoundation(int height) {
        robot.outtake.prepareForStoneDeployment();

        robot.outtake.outtakeSlides.runToPosition(height, true);

        robot.outtake.outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_OPEN_POSITION);

        sleep(500);

        robot.outtake.outtakeSlides.runToStoneHeight(robot.outtake.stoneHeight);
        robot.outtake.performStoneIntakeOperation();
    }

    private void moveToParkUnderSkyBridge(Position position) {
        robot.mecanumDrive.flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode.AudienceDirection);
        if(position == Position.Left) {
            AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.6, 3, false);
            robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
        }
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.8, 15, false);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
    }
}
