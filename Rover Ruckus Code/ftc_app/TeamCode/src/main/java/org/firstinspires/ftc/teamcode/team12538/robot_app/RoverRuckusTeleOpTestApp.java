package org.firstinspires.ftc.teamcode.team12538.robot_app;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.listeners.DetectorListener;
import com.disnodeteam.dogecv.detectors.roverruckus.SamplingOrderDetectorExt;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.team12538.components.MineralMechanism;
import org.firstinspires.ftc.teamcode.team12538.components.RobotLatch;
import org.firstinspires.ftc.teamcode.team12538.detectors.EnhancedMineralOrderDetector;
import org.firstinspires.ftc.teamcode.team12538.robotV1.AutoRobotV1;
import org.firstinspires.ftc.teamcode.team12538.robotV1.TeleOpRobotV1;
import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Robot Tele (Test)", group="Linear Opmode")
public class RoverRuckusTeleOpTestApp extends LinearOpMode implements DetectorListener {

    private ElapsedTime runtime = new ElapsedTime();

    AutoRobotV1 robot = null;
    private EnhancedMineralOrderDetector detector = null;


    @Override
    public void runOpMode() throws InterruptedException {
        try {
            OpModeUtils.init(this);
            OpModeUtils.getGlobalStore().setCloseDepoArm(true);
            OpModeUtils.getGlobalStore().setDisableInitPos(true);
            OpModeUtils.getGlobalStore().setResetArmExtensionEncoderValue(true);

            robot = new AutoRobotV1();
            robot.init();
            robot.init_imu();

            // 0.500 teleOp tilt position
            // 0.669 auto tilt position
            robot.getPhoneTilt().setPosition(robot.getAutoPhoneTiltPos());

            // detector = createDetector();
            // detector.enable();

            waitForStart();

            while (opModeIsActive()) {
                if(gamepad1.x) {
                    robot.getRobotLatch().powerLatch(-1d);
                } else if(gamepad1.b) {
                    robot.getRobotLatch().powerLatch(1d);
                } else {
                    robot.getRobotLatch().powerLatch(0d);
                }

                /*
                if (gamepad1.dpad_up) {
                    robot.getRobotLatch().powerLiftRunToPosition(1d, 2000);
                } else if (gamepad1.dpad_down) {
                    robot.getRobotLatch().powerLiftRunToPosition(1d, 0);
                }
                */

                if (gamepad1.dpad_up) {
                    robot.getRobotLatch().powerLift(1.0);
                } else if (gamepad1.dpad_down) {
                    robot.getRobotLatch().powerLift(-1.0);
                } else {
                    robot.getRobotLatch().powerLift(0d);
                }

                if(gamepad1.dpad_right) {
                    robot.getPhoneTilt().setPosition(robot.getPhoneTilt().getPosition() + 0.001);
                    // mineralMechanism.getIntakeFlip().setPosition(mineralMechanism.getIntakeFlip().getPosition() + 0.001);
                } else if(gamepad1.dpad_left) {
                    robot.getPhoneTilt().setPosition(robot.getPhoneTilt().getPosition() - 0.001);
                    // mineralMechanism.getIntakeFlip().setPosition(mineralMechanism.getIntakeFlip().getPosition() - 0.001);
                }

                if(gamepad2.dpad_left) {
                    robot.rotate(-45, 0.5, 10);
                } else if(gamepad2.dpad_right) {
                    robot.rotate(45, 0.5, 10);
                } else if(gamepad2.dpad_up) {
                    robot.moveForward(0.1, 10);
                } else if(gamepad2.dpad_down) {
                    robot.moveBackward(0.1, 10);
                }

                if(gamepad2.x) {
                    robot.strafeLeft(0.5, 5.0);
                } else if(gamepad2.b) {
                    robot.strafeRight(0.5, 5.0);
                } else if(gamepad2.y) {
                    robot.getParkingRod().setPosition(0d);
                } else if(gamepad2.a) {
                    robot.placeTeamMarker();
                }

                // telemetry.addData("lift", robot.getRobotLatch().getScissorLiftMotor().getCurrentPosition());
                // telemetry.update();
            }
        } finally {
            if(detector != null) {
                detector.disable();
            }

            OpModeUtils.stop();
        }
    }

    @Override
    public void onEvent() {
        telemetry.addData("phoneTilt", robot.getPhoneTilt().getPosition());
        telemetry.addData("Sampling Result", detector.getLastOrder());
        telemetry.update();
    }

    private EnhancedMineralOrderDetector createDetector() {
        EnhancedMineralOrderDetector detector = new EnhancedMineralOrderDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 50; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames
        // detector.areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA;
        // detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
        detector.maxAreaScorer.weight = 0.001; // if using MAX_AREA scoring
        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.yMinOffset = -60;
        detector.yMaxOffset = +30;

        detector.useDefaults();
        detector.listener = this;
        return detector;
    }
}
