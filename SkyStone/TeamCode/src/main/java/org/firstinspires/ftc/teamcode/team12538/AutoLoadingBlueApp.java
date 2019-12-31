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
        super.autoColor = AutonomousColor.Red;
        super.numSkystone = 2;
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
            AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.3, 5);
            robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
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
            AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.3, 5);
            robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
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
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 23d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 10d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.detector = robot.intakeSensor;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 15d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 12d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.TurnRight, 0.3, Math.PI/2);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;

            case Center:
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 25);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.TurnRight, 0.3, Math.PI/2);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 3d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 12d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.detector = robot.intakeSensor;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 15d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;

            case Left:
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 10d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.CurveRight, 0.5, 30d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 13d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.detector = robot.intakeSensor;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 13d);
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
        });
    }

    private void pickupSecondSkyStone(Position position) {
        // enable intake
        robot.intake.setPower(1);

        switch(position) {
            case Right:
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 1);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.timeout = 3;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.8, 54);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.timeout = 2;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 11);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.detector = robot.intakeSensor;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;

            case Center:
                gamepad.timeout = 3;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.5, 65);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.timeout = 2;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 11);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.detector = robot.intakeSensor;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;

            case Left:
                gamepad.timeout = 3;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.8, 61);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.timeout = 2;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 11);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                gamepad.detector = robot.intakeSensor;
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 5d);
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
        });

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.3, 13d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
    }

    private void crossSkyBridge(Position position, int round) {
        switch (position) {
            case Right:
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.8, round == 1 ? 35d : 50d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 14d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;

            case Center:
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.8, round == 1 ? 35d : 52d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 15d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;

            case Left:
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.8, round == 1 ? 40d : 70d, false);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 15d);
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
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.8, 15);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
    }
}
