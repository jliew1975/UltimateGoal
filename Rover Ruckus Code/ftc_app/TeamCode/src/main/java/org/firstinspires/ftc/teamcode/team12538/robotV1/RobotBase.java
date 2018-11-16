package org.firstinspires.ftc.teamcode.team12538.robotV1;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.team12538.components.MineralMechanism;
import org.firstinspires.ftc.teamcode.team12538.components.RobotLatch;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

public abstract class RobotBase extends MecanumDriveBase {
    MineralMechanism collector = null;
    RobotLatch robotLatch = null;

    Servo sheetMetal = null; // team marker mechanism

    @Override
    public void init() {
        super.init();

        HardwareMap hardwareMap = OpModeUtils.getGlobalStore().getHardwareMap();

        // Sheet Method (position: 1.0 => drop)
        sheetMetal = hardwareMap.get(Servo.class, "sheet_metal");
        sheetMetal.setPosition(0d);

        // mineral collector mechanism
        collector = new MineralMechanism(-100, 6900);
        collector.init();

        robotLatch = new RobotLatch();
        robotLatch.init();
    }

    public void controlMineralArm(double power) {
        if(collector != null) {
            collector.controlArm(power);
        }
    }

    public void prepareMineralIntake() {
        collector.flipCollectorBox(0d);
        collector.enableIntake(MineralMechanism.Direction.InTake);
    }

    public MineralMechanism getCollector() {
        return collector;
    }

    public RobotLatch getRobotLatch() { return robotLatch; }

    public void placeTeamMarker() {
        sheetMetal.setPosition(1.0);
    }
}
