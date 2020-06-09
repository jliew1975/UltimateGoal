package org.firstinspires.ftc.teamcode.team12538.opModes.eBorg;

import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.team12538.components.RobotStoneAligner;
import org.firstinspires.ftc.teamcode.team12538.components.RobotStoneClaw;
import org.firstinspires.ftc.teamcode.team12538.ext.AutoGamepad;
import org.firstinspires.ftc.teamcode.team12538.detectors.TargetPositionalDetector;
import org.firstinspires.ftc.teamcode.team12538.detectors.opencv.OpenCvDetector;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive.AutoDirection;
import org.firstinspires.ftc.teamcode.team12538.robot.SkyStoneAutoRobot;
import org.firstinspires.ftc.teamcode.team12538.utils.AutoGamepadUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousColor;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeStore;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;
import org.openftc.revextensions2.ExpansionHubEx;

import java.io.IOException;

public abstract class AutoLoadingZoneApp extends RobotApp {
    protected ElapsedTime runtime = new ElapsedTime();

    public static DriveConstraints SLOW_CONSTRAINTS = new DriveConstraints(
            30.0, 30.0, 0.0,
            Math.toRadians(180.0), Math.toRadians(180.0), 0.0
    );

    @Override
    public void performRobotOperation() throws InterruptedException {
        // Tell global store that runMode is in Autonomous mode
        OpModeUtils.getGlobalStore().runMode = OpModeStore.RunMode.Autonomous;

        // Tell global store the color of alliance
        OpModeUtils.getGlobalStore().autoColor = autoColor;

        // Reset encoder
        OpModeUtils.setResetEncoder(true);

        // Initialize a autonomous gamepad
        gamepad = new AutoGamepad();

        robot = new SkyStoneAutoRobot();
        robot.init();

        OpenCvDetector detector = new OpenCvDetector(numSkystone);
        detector.init();
        detector.activate();

        waitForStart();

        ThreadUtils.getExecutorService().submit(() -> detector.deactivate());

        try {
            if(!isStopRequested()) {
                autoVisionLogic(detector);
                sleep(1000);
            }
        } catch (IOException e) {
            e.printStackTrace();
        } finally {
            robot.intakeSensor.stop();
            OpModeUtils.stop();
        }
    }

    protected abstract void autoVisionLogic(TargetPositionalDetector detector) throws IOException;

    protected void waitForStoneDeployment() {
        while(!isStopRequested() && !stoneDeployDone) {
            // wait for stone deployment to complete
        }
    }

    protected void waitForStoneIsDetected() {
        while (!robot.intakeSensor.isDetected()) {
            // wait for stone to be detected before lowering down slide
        }
    }

    protected void prepareStoneForDeployment() {
        prepareStoneForDeployment(false);
    }

    protected void prepareStoneForDeployment(boolean isAlign) {
        ThreadUtils.getExecutorService().submit(() -> {
            if(opModeIsActive()) {
                // check if robot has 2 stones
                if(robot.intakeSensor.isDetected() && robot.intake.isStuck()) {
                    // spit out stone
                    robot.intake.setPower(-1);
                }

                if(isAlign || robot.intakeSensor.hasStoneButNotCompletelyIn()) {
                    robot.stoneAligner.setPosition(RobotStoneAligner.ALIGN);
                    sleep(500);
                    robot.stoneAligner.setPosition(RobotStoneAligner.INTAKE);
                }

                if(robot.intakeSensor.isDetected()) {
                    robot.intake.setPower(-1);
                    sleep(500);
                }

                robot.outtake.lowerSlideForStonePickup();
                robot.intake.setPower(0);
                robot.outtake.outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_OPEN_POSITION);
                sleep(300);
                robot.outtake.outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_CLOSE_POSITION);
            }
        });
    }
}
