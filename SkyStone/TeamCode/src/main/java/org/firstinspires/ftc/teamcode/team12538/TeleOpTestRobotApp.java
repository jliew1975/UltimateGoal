package org.firstinspires.ftc.teamcode.team12538;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.team12538.components.AutoGamepad;
import org.firstinspires.ftc.teamcode.team12538.components.RobotDistanceSensor;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.robot.SkyStoneAutoRobot;
import org.firstinspires.ftc.teamcode.team12538.utils.AutoGamepadUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

import java.util.Locale;

@TeleOp(name="Robot Tele (Test)", group="Linear Opmode")
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


            // RobotDistanceSensor distanceSensor = new RobotDistanceSensor("left", 2);
            // distanceSensor.init();

            // hsvValues is an array that will hold the hue, saturation, and value information.
            float hsvValues[] = {0F, 0F, 0F};

            // values is a reference to the hsvValues array.
            final float values[] = hsvValues;

            // sometimes it helps to multiply the raw RGB values with a scale factor
            // to amplify/attentuate the measured values.
            final double SCALE_FACTOR = 255;

            while (!isStarted() && !isStopRequested()) {
                telemetry.addData("Distance (cm)",
                        String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
                // send the info back to driver station using telemetry function.
                // robot.mecanumDrive.printTelemetry();
                telemetry.update();
            }

            while(opModeIsActive()) {
                if(gamepad1.a) {
                    AutoGamepadUtils.move(autoGamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.3,20);
                    robot.mecanumDrive.autoNavigateWithGamepad(autoGamepad);
                } else if (gamepad1.b) {
                    AutoGamepadUtils.move(autoGamepad, MecanumDrive.AutoDirection.StrafeRight, 0.3, 20);
                    robot.mecanumDrive.autoNavigateWithGamepad(autoGamepad);

                }
            }
        } finally {
            OpModeUtils.stop();
        }
    }
}
