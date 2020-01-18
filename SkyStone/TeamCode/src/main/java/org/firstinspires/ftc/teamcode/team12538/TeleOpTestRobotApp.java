package org.firstinspires.ftc.teamcode.team12538;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.team12538.components.RobotFoundationClaw;
import org.firstinspires.ftc.teamcode.team12538.components.RobotStoneAligner;
import org.firstinspires.ftc.teamcode.team12538.components.RobotStoneClaw;
import org.firstinspires.ftc.teamcode.team12538.detectors.opencv.OpenCvDetector;
import org.firstinspires.ftc.teamcode.team12538.ext.AutoGamepad;
import org.firstinspires.ftc.teamcode.team12538.components.RobotStoneArm;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.robot.Robot;
import org.firstinspires.ftc.teamcode.team12538.robot.SkyStoneAutoRobot;
import org.firstinspires.ftc.teamcode.team12538.utils.AutoGamepadUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.states.Button;

import java.util.Locale;

@TeleOp(name="Robot Tele (Test)", group="Linear Opmode")
@Disabled
public class TeleOpTestRobotApp extends RobotApp {
    @Override
    public void performRobotOperation() throws InterruptedException {
        try {
            AutoGamepad autoGamepad = new AutoGamepad();

            // Reset encoder values
            OpModeUtils.setResetEncoder(true);

            SkyStoneAutoRobot robot = new SkyStoneAutoRobot();
            robot.init();

            DistanceSensor sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
            ColorSensor sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

            RobotFoundationClaw foundationClaw = new RobotFoundationClaw();
            foundationClaw.init();

            // RobotDistanceSensor distanceSensor = new RobotDistanceSensor("left", 2);
            // distanceSensor.init();
            int stoneHeight = 1;

            Button btnX = new Button();
            Button btnB = new Button();


            waitForStart();

            while(opModeIsActive()) {
                if(gamepad1.right_trigger > 0d) {
                    robot.outtake.aligner.setPosition(RobotStoneAligner.ALIGN);
                } else {
                    robot.outtake.aligner.setPosition(RobotStoneAligner.INTAKE);
                }

                /*
                if(gamepad1.x) {
                    AutoGamepadUtils.move(autoGamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.6, 10d);
                    AutoGamepadUtils.move(autoGamepad, MecanumDrive.AutoDirection.TurnLeft, 0.3, Math.PI/2);
                    // autoGamepad.resetAngle = true;
                    // autoGamepad.backCurving = true;
                    // AutoGamepadUtils.move(autoGamepad, MecanumDrive.AutoDirection.CurveLeft, 0.6, 30d);
                    robot.mecanumDrive.autoNavigateWithGamepad(autoGamepad);
                } else if (gamepad1.b) {
                    AutoGamepadUtils.move(autoGamepad, MecanumDrive.AutoDirection.StrafeRight, 0.6, 10d);
                    // AutoGamepadUtils.move(autoGamepad, MecanumDrive.AutoDirection.TurnRight, 0.3, Math.PI/2);
                    // autoGamepad.resetAngle = true;
                    // autoGamepad.backCurving = true;
                    // AutoGamepadUtils.move(autoGamepad, MecanumDrive.AutoDirection.CurveRight, 0.6, 30d);
                    robot.mecanumDrive.autoNavigateWithGamepad(autoGamepad);
                } else if(gamepad1.y) {
                    AutoGamepadUtils.move(autoGamepad, MecanumDrive.AutoDirection.Forward, 0.6, 16d);
                    robot.mecanumDrive.autoNavigateWithGamepad(autoGamepad);
                } else if(gamepad1.a) {
                    AutoGamepadUtils.move(autoGamepad, MecanumDrive.AutoDirection.Backward, 0.6, 16d);
                    robot.mecanumDrive.autoNavigateWithGamepad(autoGamepad);
                }
                */


                btnB.input(gamepad1.b);
                btnX.input(gamepad1.x);

                if(btnB.onPress()) {
                    robot.outtake.prepareForStoneDeployment();
                }

                if(btnX.onPress()) {
                    robot.outtake.performStoneIntakeOperation();
                }

                robot.slidesSensor.printTelemetry();
                telemetry.update();
            }
        } finally {
            OpModeUtils.stop();
        }
    }
}
