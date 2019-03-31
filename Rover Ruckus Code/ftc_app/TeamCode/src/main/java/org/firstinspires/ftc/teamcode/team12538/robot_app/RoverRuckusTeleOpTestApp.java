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
public class RoverRuckusTeleOpTestApp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtils.init(this);
        OpModeUtils.getGlobalStore().setCloseDepoArm(false);
        OpModeUtils.getGlobalStore().setDisableInitPos(true);
        OpModeUtils.getGlobalStore().setResetArmExtensionEncoderValue(false);

        try {
            TeleOpRobotV1 robot = new TeleOpRobotV1();
            robot.init();

            waitForStart();

            if(isStopRequested()) {
                return;
            }

            // tilt the phone for teleOp mode
            robot.getParkingRod().setPosition(1d);
            robot.getPhoneTilt().setPosition(robot.telePhoneTiltPos);

            while (opModeIsActive()) {
                robot.player1controls(gamepad1);
                robot.player2Controls(gamepad2);

                // robot.getCollector().printTelemetry();
                robot.getRobotLatch().printTelemetry();
                telemetry.update();
            }
        } finally {
            OpModeUtils.stop();
        }
    }
}
