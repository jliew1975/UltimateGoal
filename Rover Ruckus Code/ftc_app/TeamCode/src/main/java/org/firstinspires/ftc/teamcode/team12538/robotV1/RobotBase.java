package org.firstinspires.ftc.teamcode.team12538.robotV1;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team12538.components.MineralMechanism;
import org.firstinspires.ftc.teamcode.team12538.components.RobotLatch;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

import lombok.Data;
import lombok.EqualsAndHashCode;

import static org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils.sleep;

@Data
@EqualsAndHashCode(callSuper = true)
public abstract class RobotBase extends MecanumDriveBase {
    MineralMechanism collector = null;
    RobotLatch robotLatch = null;

    private Servo phoneTilt = null;
    private Servo parkingRod = null;

    public double telePhoneTiltPos = 0.500;
    public double autoPhoneTiltPos = 0.669;

    @Override
    public void init() {
        super.init();

        HardwareMap hardwareMap = OpModeUtils.getGlobalStore().getHardwareMap();

        // phone tilting servo initialization
        phoneTilt = hardwareMap.get(Servo.class, "phone_tilt");
        phoneTilt.setPosition(autoPhoneTiltPos);

        // parking rod initialization
        parkingRod = hardwareMap.get(Servo.class, "parking_rod");
        if(!OpModeUtils.getGlobalStore().isDisableInitPos()) {
            parkingRod.setPosition(1d);
        }

        // mineral collector mechanism initialization
        collector = new MineralMechanism();
        collector.init();

        // latching mechanism initialization
        robotLatch = new RobotLatch();
        robotLatch.init();
    }

    public void driveRobot(Gamepad gamepad) {
        // Drive mode for mecanum wheel
        double r = Math.hypot(gamepad.left_stick_x, gamepad.left_stick_y);
        double robotAngle = Math.atan2(gamepad.left_stick_y, gamepad.left_stick_x) - Math.PI / 4;
        double rightX = gamepad.right_stick_x;

        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
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

    public void placeTeamMarker() {
        collector.flipCollectorBox(collector.intakeFlipPrepPos);
        sleep(500);
        collector.enableIntake(MineralMechanism.Direction.OutTake);
        sleep(1000);
        collector.flipCollectorBox(collector.intakeFlipUpPos);
        collector.disableIntake();
    }
}
