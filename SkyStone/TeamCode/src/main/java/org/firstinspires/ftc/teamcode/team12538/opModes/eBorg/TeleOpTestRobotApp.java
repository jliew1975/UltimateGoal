package org.firstinspires.ftc.teamcode.team12538.opModes.eBorg;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.team12538.components.RobotFoundationClaw;
import org.firstinspires.ftc.teamcode.team12538.components.RobotStoneAligner;
import org.firstinspires.ftc.teamcode.team12538.components.RobotStoneClaw;
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

            waitForStart();

            Button dPadUp = new Button();
            Button dPadDown = new Button();

            while(opModeIsActive()) {
                dPadUp.input(gamepad1.dpad_up);
                dPadDown.input(gamepad1.dpad_down);

                if (dPadUp.onPress()) {
                    robot.capstone.setPosition(robot.capstone.getPosition() + 0.05);
                } else if (dPadDown.onPress()) {
                    robot.capstone.setPosition(robot.capstone.getPosition() - 0.05);
                }

                if (gamepad1.right_bumper) {
                    robot.intake.setPower(1);
                } else if (gamepad1.left_bumper) {
                    robot.intake.setPower(-1);
                } else {
                    robot.intake.setPower(0);
                }

                telemetry.addData("Capstone", robot.capstone.getPosition());
                telemetry.addData("Stone Distance", robot.intakeSensor.sensorDistance.getDistance(DistanceUnit.INCH));
                telemetry.addData("left intake Power Draw", robot.intake.getLeftRoller().getCurrentPowerDraw());
                telemetry.addData("right intake Power Draw", robot.intake.getRightRoller().getCurrentPowerDraw());
                telemetry.update();
            }
        } finally {
            OpModeUtils.stop();
        }
    }
}
