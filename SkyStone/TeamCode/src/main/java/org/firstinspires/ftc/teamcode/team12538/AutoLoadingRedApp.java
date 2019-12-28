package org.firstinspires.ftc.teamcode.team12538;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.team12538.components.RobotStoneClaw;
import org.firstinspires.ftc.teamcode.team12538.detectors.TargetPositionalDetector;
import org.firstinspires.ftc.teamcode.team12538.detectors.TargetPositionalDetector.Position;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.utils.AutoGamepadUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousColor;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousMode;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

@Autonomous(name="Loading Red", group="Linear Opmode")
@Disabled
public class AutoLoadingRedApp extends AutoLoadingZoneApp {
    public AutoLoadingRedApp() {
        super();
        super.autoMode = AutonomousMode.RedLoading;
        super.autoColor = AutonomousColor.Red;
    }

    @Override
    protected void autoVuforiaLogic(TargetPositionalDetector detector) {
        Position skystonePosition = detector.getPosition();

        switch (skystonePosition) {
            case Left:
                executeLeftLogic();
                break;
            case Center:
                executeCenterLogic();
                break;
            default:
                executeRightLogic();
        }

        sleep(500);
    }

    protected void executeLeftLogic() {
        pickupSkyStone(Position.Left);

        crossSkyBridge(Position.Left);

        moveFoundationToBuildingSite();

        deploySkyStoneToFoundation();

        moveToParkUnderSkyBridge();
    }

    protected void executeCenterLogic() {
        pickupSkyStone(Position.Center);

        crossSkyBridge(Position.Center);

        moveFoundationToBuildingSite();

        deploySkyStoneToFoundation();

        // moveToParkUnderSkyBridge();
    }

    protected void executeRightLogic() {
        pickupSkyStone(Position.Right);

        crossSkyBridge(Position.Right);

        moveFoundationToBuildingSite();

        deploySkyStoneToFoundation();

        // moveToParkUnderSkyBridge();
    }

    private void pickupSkyStone(Position position) {
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.3, 20d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(200);

        switch(position) {
            case Right:
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.TurnRight, 0.3, Math.PI / 2);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                sleep(200);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, getDistanceToBackoffForSkystone(position));
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                sleep(200);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 18d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                sleep(200);
                break;

            case Left:
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.TurnLeft, 0.3, Math.PI / 2);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                sleep(200);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, getDistanceToBackoffForSkystone(position));
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                sleep(200);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 18d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                sleep(200);
                break;
        }

        ThreadUtils.getExecutorService().submit(() -> robot.outtake.outtakeSlides.runToPosition(40));

        // enable intake
        robot.intake.setPower(1);

        // start the distance sensor for stone detection
        robot.intakeSensor.start();

        sleep(500);

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
    }

    private void crossSkyBridge(Position position) {
        switch (position) {
            case Right:
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 12d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                sleep(200);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.5, 50d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 20d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;
            case Center:
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 12d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                sleep(200);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.5, 50d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 10d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;
            case Left:
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.5, 12d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                sleep(200);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.5, 60d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 10d);
                robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
                break;
        }

        sleep(500);
    }

    private void moveFoundationToBuildingSite() {
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.TurnLeft, 0.3, Math.PI/2);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(200);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.3, 8);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.foundationClaw.lowerClaw();

        // At the same time have the stone out ready for deployment.
        ThreadUtils.getExecutorService().submit(() -> robot.outtake.prepareForStoneDeployment());

        sleep(800);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.6, 2);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(200);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.CurveRight, 0.6, 30);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(200);

        gamepad.resetAngle = true;
        gamepad.timeout = 1d;
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Backward, 0.5, 15);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        robot.foundationClaw.raiseClaw();
    }

    private void deploySkyStoneToFoundation() {
        // lower slide for stone deployment
        ThreadUtils.getExecutorService().submit(() -> robot.outtake.outtakeSlides.runToPosition(20, true));
        sleep(500);

        robot.outtake.outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_OPEN_POSITION);

        sleep(500);

        robot.outtake.outtakeSlides.runToStoneHeight(robot.outtake.stoneHeight);

        sleep(500);

        robot.outtake.performStoneIntakeOperation();
    }

    private void moveToParkUnderSkyBridge() {
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.StrafeRight, 0.5, 5d);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);

        sleep(200);

        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.5, 30);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
    }

    private double getDistanceToBackoffForSkystone(Position position) {
        if(position == Position.Center) {
            return 10d;
        }

        return 5d;
    }

    private double getDistanceToFoundation(Position position) {
        if(position == Position.Center) {
            return 10d;
        }

        return 5d;
    }
}
