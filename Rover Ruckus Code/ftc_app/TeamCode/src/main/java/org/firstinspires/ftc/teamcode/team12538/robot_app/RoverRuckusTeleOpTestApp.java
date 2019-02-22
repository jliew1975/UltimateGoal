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
import org.firstinspires.ftc.teamcode.team12538.robotV1.AutoRobotV1;
import org.firstinspires.ftc.teamcode.team12538.robotV1.TeleOpRobotV1;
import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

@TeleOp(name="Robot Tele (Test)", group="Linear Opmode")
public class RoverRuckusTeleOpTestApp extends LinearOpMode implements DetectorListener {

    private ElapsedTime runtime = new ElapsedTime();

    private Object lock = new Object();
    private volatile boolean busy = false;

    private Servo phoneTilt;
    private RobotLatch robotLatch;
    private MineralMechanism mineralMechanism;

    private SamplingOrderDetectorExt detector = null;

    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor rearRightDrive = null;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            OpModeUtils.init(this);
            OpModeUtils.getGlobalStore().setCloseDepoArm(true);
            OpModeUtils.getGlobalStore().setDisableInitPos(true);
            OpModeUtils.getGlobalStore().setResetArmExtensionEncoderValue(true);

            robotLatch = new RobotLatch();
            robotLatch.init();

            mineralMechanism = new MineralMechanism();
            mineralMechanism.init();

            phoneTilt = hardwareMap.get(Servo.class, "phone_tilt");
            phoneTilt.setPosition(0.500);

            initDrive(hardwareMap);

            // detector = createDetector();
            // detector.enable();

            // 0.500 verticle
            // 0.647 scan position
            waitForStart();

            while (opModeIsActive()) {
                moveRobot(gamepad1);

                if(gamepad1.x) {
                    robotLatch.latchClose();
                } else if(gamepad1.b) {
                    robotLatch.latchOpen();
                } else if(gamepad1.a) {
                    // robotLatch.unlatch();
                } if(!robotLatch.isScissorLiftBusy()) {
                    if (gamepad1.dpad_up) {
                        robotLatch.powerLift(-1.0, 8000);
                    } else if (gamepad1.dpad_down) {
                        robotLatch.powerLift(1.0);
                    } else {
                        robotLatch.powerLift(0d);
                    }
                }

                // Intake controls
                if(gamepad2.right_bumper) {
                    mineralMechanism.enableIntake(MineralMechanism.Direction.InTake);
                } else if(gamepad2.left_bumper) {
                    mineralMechanism.enableIntake(MineralMechanism.Direction.OutTake);
                } else {
                    mineralMechanism.disableIntake();
                }

                /*
                if(gamepad1.dpad_up) {
                    phoneTilt.setPosition(phoneTilt.getPosition()+0.001);
                } else if(gamepad1.dpad_down) {
                    phoneTilt.setPosition(phoneTilt.getPosition()-0.001);
                }
                */

                if(gamepad2.y) {
                    mineralMechanism.flipCollectorBox(mineralMechanism.intakeFlipUpPos);
                } else if(gamepad2.a) {
                    mineralMechanism.flipCollectorBox(mineralMechanism.intakeFlipDownPos);
                } else if(gamepad2.x) {
                    mineralMechanism.flipCollectorBox(mineralMechanism.intakeFlipPrepPos);
                }

                if(gamepad2.b) {
                    // auto deposit minerals to outtake box
                    mineralMechanism.autoMineralDeposit();
                } else {
                    // arm extension control
                    mineralMechanism.controlArmExt(-gamepad2.left_stick_x);
                }

                if(gamepad2.dpad_up) {
                    mineralMechanism.liftDepo(3330);
                } else if(gamepad2.dpad_down){
                    mineralMechanism.lowerDepo();
                } else if(gamepad2.right_trigger > 0d && mineralMechanism.canFlipDepoBox()) {
                    mineralMechanism.rotateDepositBox(1d);
                } else if(gamepad2.dpad_right) {
                    mineralMechanism.getIntakeFlip().setPosition(mineralMechanism.getIntakeFlip().getPosition() + 0.001);
                } else if(gamepad2.dpad_left){
                    mineralMechanism.getIntakeFlip().setPosition(mineralMechanism.getIntakeFlip().getPosition() - 0.001);
                }

                // robotLatch.printTelemetry();
                mineralMechanism.printTelemetry();
                // telemetry.addData("phoneTilt", phoneTilt.getPosition());
                telemetry.update();
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
        telemetry.addData("phoneTilt", phoneTilt.getPosition());
        telemetry.addData("Sampling Result", detector.getLastOrder());
        telemetry.update();
    }

    private SamplingOrderDetectorExt createDetector() {
        SamplingOrderDetectorExt detector = new SamplingOrderDetectorExt();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 150; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames
        // detector.areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA;
        // detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
        detector.maxAreaScorer.weight = 0.001; // if using MAX_AREA scoring
        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.useDefaults();
        detector.listener = this;
        return detector;
    }

    private void initDrive(HardwareMap hardwareMap) {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "left_forward_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "right_forward_drive");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rearRightDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);
    }

    private void moveRobot(Gamepad gamepad) {
        // Drive mode for mecanum wheel
        double r = Math.hypot(gamepad.left_stick_x, gamepad.left_stick_y);
        double robotAngle = Math.atan2(gamepad.left_stick_y, gamepad.left_stick_x) - Math.PI / 4;
        double rightX = gamepad.right_stick_x;

        final double v1 = r * Math.sin(robotAngle) - rightX;
        final double v2 = r * Math.cos(robotAngle) + rightX;
        final double v3 = r * Math.cos(robotAngle) - rightX;
        final double v4 = r * Math.sin(robotAngle) + rightX;

        double power = 1.0;
        if(gamepad.left_trigger > 0 || gamepad.right_trigger > 0) {
            power = 0.3; // slowdown robot on left or right trigger
        }

        frontLeftDrive.setPower(power * Math.signum(v1));
        frontRightDrive.setPower(power * Math.signum(v2));
        rearLeftDrive.setPower(power * Math.signum(v3));
        rearRightDrive.setPower(power * Math.signum(v4));
    }
}
