package org.firstinspires.ftc.teamcode.team12538.robotV1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

import edu.spa.ftclib.internal.drivetrain.MecanumDrivetrain;

public class AutoRobotTest extends AutoRobotV1 {
    Telemetry telemetry = null;

    @Override
    public void init() {
        super.init();
        super.init_imu();
        telemetry = OpModeUtils.getGlobalStore().getTelemetry();
    }

    public void controlWithGamePad(Gamepad gamepad) {
        // Drive mode for mecanum wheel
        double r = Math.hypot(gamepad.left_stick_x, gamepad.left_stick_y);
        double robotAngle = Math.atan2(gamepad.left_stick_y, gamepad.left_stick_x) - Math.PI / 4;
        double rightX = gamepad.right_stick_x;

        final double v1 = r * Math.sin(robotAngle) - rightX;
        final double v2 = r * Math.cos(robotAngle) + rightX;
        final double v3 = r * Math.cos(robotAngle) - rightX;
        final double v4 = r * Math.sin(robotAngle) + rightX;

        double power = 1.0;
        if(gamepad.left_trigger > 0) {
            power = 0.3; // slowdown robot on left trigger
        }

        frontLeftDrive.setPower(power * Math.signum(v1));
        frontRightDrive.setPower(power * Math.signum(v2));
        rearLeftDrive.setPower(power * Math.signum(v3));
        rearRightDrive.setPower(power * Math.signum(v4));
    }

    public void printMotorTelemetry() {
        if(telemetry == null) {
            return;
        }

        telemetry.addData("Motor Position", "Motor at %7d :%7d",
                motors.get(0).getCurrentPosition(),
                motors.get(1).getCurrentPosition(),
                motors.get(2).getCurrentPosition(),
                motors.get(3).getCurrentPosition());
        telemetry.update();
    }
}
