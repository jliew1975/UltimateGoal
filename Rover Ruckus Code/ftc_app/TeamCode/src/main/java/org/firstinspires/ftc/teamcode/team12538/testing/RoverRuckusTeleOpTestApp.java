package org.firstinspires.ftc.teamcode.team12538.testing;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.team12538.components.MineralMechanism;
import org.firstinspires.ftc.teamcode.team12538.detectors.EnhancedMineralOrderDetector;
import org.firstinspires.ftc.teamcode.team12538.detectors.MineralDetector;
import org.firstinspires.ftc.teamcode.team12538.robotV1.AutoRobotV1;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

@TeleOp(name="Robot Tele (Test)", group="Linear Opmode")
public class RoverRuckusTeleOpTestApp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    MineralDetector detector = null;

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtils.init(this);
        OpModeUtils.getGlobalStore().setCloseDepoArm(true);
        OpModeUtils.getGlobalStore().setDisableInitPos(false);
        OpModeUtils.getGlobalStore().setResetArmExtensionEncoderValue(true);

        try {
            AutoRobotV1 robot = new AutoRobotV1();
            robot.init();
            robot.init_imu();

            robot.getCameraTilt().setPosition(0.94);

            // detector = createDetector();
            // detector.enable();

            waitForStart();

            if(isStopRequested()) {
                return;
            }

            Servo depo = robot.getCollector().getDepo();
            while(opModeIsActive()) {
                if (gamepad1.dpad_up) {
                    robot.getCameraTilt().setPosition(robot.getCameraTilt().getPosition() + 0.0001);
                } else if (gamepad1.dpad_down) {
                    robot.getCameraTilt().setPosition(robot.getCameraTilt().getPosition() - 0.0001);
                }

                if(gamepad2.dpad_up) {
                    depo.setPosition(depo.getPosition() - 0.0001);
                } else if(gamepad2.dpad_down) {
                    depo.setPosition(depo.getPosition() + 0.0001);
                }

                if(gamepad1.x) {
                    robot.moveForward(0.3, 10);
                    robot.corneringLeft(0.3, -20, false);
                    robot.moveForward(0.3, 10, true);
                }

                telemetry.addData("Camera Tilt", robot.getCameraTilt().getPosition());
                telemetry.addData("Depo", depo.getPosition());
                telemetry.update();
            }
        } finally {
            if(detector != null) {
                detector.disable();
            }

            OpModeUtils.stop();
        }
    }

    protected EnhancedMineralOrderDetector createDetector() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        EnhancedMineralOrderDetector detector = new EnhancedMineralOrderDetector();
        detector.VUFORIA_KEY = "AWbfTmn/////AAABmY0xuIe3C0RHvL3XuzRxyEmOT2OekXBSbqN2jot1si3OGBObwWadfitJR/D6Vk8VEBiW0HG2Q8UAEd0//OliF9aWCRmyDJ1mMqKCJZxpZemfT5ELFuWnJIZWUkKyjQfDNe2RIaAh0ermSxF4Bq77IDFirgggdYJoRIyi2Ys7Gl9lD/tSonV8OnldIN/Ove4/MtEBJTKHqjUEjC5U2khV+26AqkeqbxhFTNiIMl0LcmSSfugGhmWFGFtuPtp/+flPBRGoBO+tSl9P2sV4mSUBE/WrpHqB0Jd/tAmeNvbtgQXtZEGYc/9NszwRLVNl9k13vrBcgsiNxs2UY5xAvA4Wb6LN7Yu+tChwc+qBiVKAQe09\n";
        detector.init(hardwareMap.appContext,CameraViewDisplay.getInstance(), DogeCV.CameraMode.WEBCAM, true, webcamName);

        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = -50; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames
        // detector.areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA;
        // detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
        detector.maxAreaScorer.weight = 0.001; // if using MAX_AREA scoring
        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.yMinOffset = -160;
        detector.yMaxOffset = 110;

        detector.useDefaults();
        return detector;
    }
}
