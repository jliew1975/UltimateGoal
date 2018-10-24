package org.firstinspires.ftc.teamcode.team12538.robotV1;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.team12538.components.MineralCollector;
import org.firstinspires.ftc.teamcode.team12538.drive.MacanumDriveBase;

public abstract class RobotBase extends MacanumDriveBase {
    MineralCollector collector = null;

    @Override
    public void init() {
        super.init();

        // mineral collector mechanism
        collector = new MineralCollector(0, 10500);
        collector.init();
    }

    public void controlMineralArm(double power) {
        if(collector != null) {
            collector.controlArm(power);
        }
    }

    public void turnOnIntake(MineralCollector.Direction direction) {
        collector.enableIntake(direction);
    }

    public void turnOffIntake() {
        collector.disableIntake();
    }

    public void lowerArm() {
        collector.lowerArm();
    }

    public void lowerArm(double power) {
        collector.lowerArm(power);
    }

    public void liftArm() {
        collector.liftArm();
    }

    public void liftArm(double power) {
        collector.liftArm(power);
    }

    public void placeTeamMarker() {

    }
}
