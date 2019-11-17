package org.firstinspires.ftc.teamcode.team12538;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.team12538.components.AutoGamepad;
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

            // hsvValues is an array that will hold the hue, saturation, and value information.
            float hsvValues[] = {0F, 0F, 0F};

            // values is a reference to the hsvValues array.
            final float values[] = hsvValues;

            // sometimes it helps to multiply the raw RGB values with a scale factor
            // to amplify/attentuate the measured values.
            final double SCALE_FACTOR = 255;

            while (!isStarted() && !isStopRequested()) {
                Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                        (int) (sensorColor.green() * SCALE_FACTOR),
                        (int) (sensorColor.blue() * SCALE_FACTOR),
                        hsvValues);

                // send the info back to driver station using telemetry function.
                telemetry.addData("Distance (cm)",
                        String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
                telemetry.addData("Alpha", sensorColor.alpha());
                telemetry.addData("Red  ", sensorColor.red());
                telemetry.addData("Green", sensorColor.green());
                telemetry.addData("Blue ", sensorColor.blue());
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.update();
            }

            if(isStopRequested()) {
                return;
            }

            double power = 0.3d;

            while (opModeIsActive()) {

                if(gamepad1.a) {
                    AutoGamepadUtils.move(autoGamepad, MecanumDrive.AutoDirection.Backward, power,20.0);
                    robot.mecanumDrive.autoNavigateWithGamepad(autoGamepad);
                } else if(gamepad1.y) {
                    AutoGamepadUtils.move(autoGamepad, MecanumDrive.AutoDirection.Forward, power,20.0);
                    robot.mecanumDrive.autoNavigateWithGamepad(autoGamepad);
                } else if(gamepad1.x) {
                    AutoGamepadUtils.move(autoGamepad, MecanumDrive.AutoDirection.StrafeLeft, power,10.0);
                    robot.mecanumDrive.autoNavigateWithGamepad(autoGamepad);
                    // AutoGamepadUtils.turn(autoGamepad, MecanumDrive.AutoDirection.TurnLeft, 0.2, 45);
                    // robot.mecanumDrive.autoNavigateWithGamepad(autoGamepad);

                    // AutoGamepadUtils.turn(autoGamepad, MecanumDrive.AutoDirection.TurnLeft, 0.3, 45);
                    // robot.mecanumDrive.rotateUsingIMU(autoGamepad);
                } else if(gamepad1.b) {
                    AutoGamepadUtils.move(autoGamepad, MecanumDrive.AutoDirection.StrafeRight, power,10.0);
                    robot.mecanumDrive.autoNavigateWithGamepad(autoGamepad);
                    // AutoGamepadUtils.turn(autoGamepad, MecanumDrive.AutoDirection.TurnRight, 0.2, 45);
                    // robot.mecanumDrive.autoNavigateWithGamepad(autoGamepad);
                    // AutoGamepadUtils.turn(autoGamepad, MecanumDrive.AutoDirection.TurnRight, 0.3, 45);
                    // robot.mecanumDrive.rotateUsingIMU(autoGamepad);
                }

                robot.mecanumDrive.printTelemetry();
                telemetry.update();

                /*
                if(gamepad1.y) {
                    robot.autoStoneArm.setPosition(1d);
                } else if(gamepad1.a) {
                    robot.autoStoneArm.setPosition(0.45);
                }
                */
            }
        } finally {
            OpModeUtils.stop();
        }
    }
}
