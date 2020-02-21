package org.firstinspires.ftc.teamcode.team12538.opModes.eBorg;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

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

            while(opModeIsActive()) {
                if(gamepad1.x) {
                    robot.outtake.lowerSlideForStonePickup();
                } else if(gamepad1.b) {
                    robot.outtake.outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_INTAKE_POSITION);
                } else if(gamepad1.a) {
                    robot.stoneAligner.setPosition(RobotStoneAligner.ALIGN);
                } else if(gamepad1.y) {
                    robot.stoneAligner.setPosition(RobotStoneAligner.INTAKE);
                }

                telemetry.addData("Outtake Aem", robot.outtake.outtakeClaw.getArmPosition());
                telemetry.update();
            }
        } finally {
            OpModeUtils.stop();
        }
    }
}
