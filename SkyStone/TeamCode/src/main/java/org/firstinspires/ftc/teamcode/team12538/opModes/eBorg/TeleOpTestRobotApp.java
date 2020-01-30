package org.firstinspires.ftc.teamcode.team12538.opModes.eBorg;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.team12538.components.RobotFoundationClaw;
import org.firstinspires.ftc.teamcode.team12538.ext.AutoGamepad;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.robot.SkyStoneAutoRobot;
import org.firstinspires.ftc.teamcode.team12538.utils.AutoGamepadUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.states.Button;

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

            RobotFoundationClaw foundationClaw = new RobotFoundationClaw();
            foundationClaw.init();

            // RobotDistanceSensor distanceSensor = new RobotDistanceSensor("left", 2);
            // distanceSensor.init();
            int stoneHeight = 1;

            Button btnX = new Button();
            Button btnB = new Button();

            while(!isStopRequested() && !isStarted()) {
                robot.mecanumDrive.printTelemetry();
                telemetry.update();
            }

            waitForStart();

            while(opModeIsActive()) {
                if(gamepad1.x) {
                    // AutoGamepadUtils.move(autoGamepad, MecanumDrive.AutoDirection.StrafeLeft, 0.6, 10d);
                    AutoGamepadUtils.move(autoGamepad, MecanumDrive.AutoDirection.TurnLeft, 0.3, Math.PI/2);
                    // autoGamepad.resetAngle = true;
                    // autoGamepad.backCurving = true;
                    // AutoGamepadUtils.move(autoGamepad, MecanumDrive.AutoDirection.CurveLeft, 0.6, 30d);
                    robot.mecanumDrive.autoNavigateWithGamepad(autoGamepad);
                } else if (gamepad1.b) {
                    // AutoGamepadUtils.move(autoGamepad, MecanumDrive.AutoDirection.StrafeRight, 0.6, 10d);
                    AutoGamepadUtils.move(autoGamepad, MecanumDrive.AutoDirection.TurnRight, 0.3, Math.PI/2);
                    // autoGamepad.resetAngle = true;
                    // autoGamepad.backCurving = true;
                    // AutoGamepadUtils.move(autoGamepad, MecanumDrive.AutoDirection.CurveRight, 0.6, 30d);
                    robot.mecanumDrive.autoNavigateWithGamepad(autoGamepad);
                } else if(gamepad1.y) {
                    AutoGamepadUtils.move(autoGamepad, MecanumDrive.AutoDirection.Forward, 0.3, 60d);
                    robot.mecanumDrive.autoNavigateWithGamepad(autoGamepad);
                } else if(gamepad1.a) {
                    AutoGamepadUtils.move(autoGamepad, MecanumDrive.AutoDirection.Backward, 0.3, 60d);
                    robot.mecanumDrive.autoNavigateWithGamepad(autoGamepad);
                }

                Position position = robot.mecanumDrive.getPosition();
                telemetry.addData("Current", "(%.3f %.3f %.3f)%s", position.x, position.y, position.z, position.unit);
                telemetry.update();
            }
        } finally {
            OpModeUtils.stop();
        }
    }
}
