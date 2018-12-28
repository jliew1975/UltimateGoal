package org.firstinspires.ftc.teamcode.team12538.robotV1;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.team12538.components.MineralMechanism;
import org.firstinspires.ftc.teamcode.team12538.components.RobotLatch;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

import lombok.Data;
import lombok.EqualsAndHashCode;

import static org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils.sleep;

@Data
@EqualsAndHashCode(callSuper = true)
public abstract class RobotBase extends MecanumDriveBase {
    MineralMechanism collector = null;
    RobotLatch robotLatch = null;

    Servo phoneTilt = null;

    @Override
    public void init() {
        super.init();

        HardwareMap hardwareMap = OpModeUtils.getGlobalStore().getHardwareMap();

        phoneTilt = hardwareMap.get(Servo.class, "phone_tilt");
        phoneTilt.setPosition(0.74);

        // mineral collector mechanism
        collector = new MineralMechanism(-100, 5000);
        collector.init();

        robotLatch = new RobotLatch();
        robotLatch.init();
    }

    public void prepareMineralIntake() {
        collector.flipCollectorBox(0d);
        collector.enableIntake(MineralMechanism.Direction.InTake);
    }

    public void placeTeamMarker() {
        // collector.positionArm(800);
        collector.flipCollectorBox(0.75);
        sleep(500);
        collector.enableIntake(MineralMechanism.Direction.OutTake);
        sleep(500);
        collector.flipCollectorBox(0.2);
        collector.disableIntake();
        // collector.positionArm(0);
    }
}
