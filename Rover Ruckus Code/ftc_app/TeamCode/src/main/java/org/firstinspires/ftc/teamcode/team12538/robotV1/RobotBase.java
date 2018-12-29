package org.firstinspires.ftc.teamcode.team12538.robotV1;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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

    Servo phoneTilt = null;
    Servo parkingRod = null;

    @Override
    public void init() {
        super.init();

        HardwareMap hardwareMap = OpModeUtils.getGlobalStore().getHardwareMap();

        // phone tilting servo initialization
        phoneTilt = hardwareMap.get(Servo.class, "phone_tilt");
        phoneTilt.setPosition(0.74);

        // parking rod initialization
        parkingRod = hardwareMap.get(Servo.class, "parking_rod");
        parkingRod.setPosition(0d);

        // mineral collector mechanism initialization
        collector = new MineralMechanism(-100, 5000);
        collector.init();

        // latching mechanism initialization
        robotLatch = new RobotLatch();
        robotLatch.init();
    }

    public void placeTeamMarker() {
        collector.flipCollectorBox(0.75);
        sleep(500);
        collector.enableIntake(MineralMechanism.Direction.OutTake);
        sleep(500);
        collector.flipCollectorBox(0.2);
        collector.disableIntake();
    }
}
