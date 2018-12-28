package org.firstinspires.ftc.teamcode.team12538.robot_app;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverruckus.SamplingOrderDetectorExt;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.team12538.robotV1.AutoRobotTest;
import org.firstinspires.ftc.teamcode.team12538.robotV1.AutoRobotV1;
import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

@TeleOp(name="Robot Tele (Test)", group="Linear Opmode")
@Disabled
public class RoverRuckusTeleOpTestApp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private Object lock = new Object();
    private volatile boolean busy = false;

    Servo hook = null;
    Servo phoneTilt = null;

    AutoRobotV1 robot = null;
    SamplingOrderDetectorExt detector = null;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            OpModeUtils.init(this);
            OpModeUtils.getGlobalStore().setCloseDepoArm(true);
            OpModeUtils.getGlobalStore().setDisableInitPos(false);
            OpModeUtils.getGlobalStore().setResetArmExtensionEncoderValue(true);

            robot = new AutoRobotV1();
            robot.init();
            robot.init_imu();

            hook = hardwareMap.get(Servo.class, "latch");
            hook.setPosition(0.1);

            phoneTilt = hardwareMap.get(Servo.class, "phone_tilt");
            phoneTilt.setPosition(0.74);

            Servo leftArm = hardwareMap.servo.get("l_flip");
            Servo rightArm = hardwareMap.servo.get("r_flip");
            rightArm.setDirection(Servo.Direction.REVERSE);
            leftArm.setPosition(0.2);
            rightArm.setPosition(0.2);

            detector = createDetector();
            detector.enable();

            waitForStart();

            while (opModeIsActive()) {
                if(gamepad1.dpad_up) {
                    phoneTilt.setPosition(phoneTilt.getPosition() + 0.01);
                } else if(gamepad1.dpad_down) {
                    phoneTilt.setPosition(phoneTilt.getPosition() - 0.01);
                }

                if(gamepad1.dpad_left) {
                    robot.rotate(30, 0.1, 5.0, detector);
                } else if(gamepad1.dpad_right) {
                    robot.rotate(-30, 0.1, 5.0, detector);
                }

                if(gamepad1.y) {
                    robot.placeTeamMarker();
                }

                telemetry.addData("phoneTilt", phoneTilt.getPosition());
                telemetry.update();
            }
        } finally {
            if(detector != null) {
                detector.disable();
            }

            OpModeUtils.stop();
        }
    }

    protected SamplingOrderDetectorExt createDetector() {
        SamplingOrderDetectorExt detector = new SamplingOrderDetectorExt();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        detector.alignSize = 80; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 80; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames
        // detector.areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA;
        // detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
        detector.maxAreaScorer.weight = 0.001; // if using MAX_AREA scoring
        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.useDefaults();

        return detector;
    }
}
