package org.firstinspires.ftc.teamcode.team12538.robot_app;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto (Crater) - Park Only)", group="Linear Opmode")
public class AutoCraterParkOnly extends AutoCrater {
    public AutoCraterParkOnly() {
        super.isMultiMineral = true;
    }

    protected void collectMineralOffTapedArea(MineralLocation mineralLocation) throws InterruptedException {
        super.collectMineralOffTapedArea(mineralLocation);
    }

    @Override
    protected void navigateToDepot(MineralLocation mineralLocation) throws InterruptedException {
        // intentionally left blank for no-op
    }

    @Override
    protected void navigateForParking(MineralLocation mineralLocation) {
        robot.moveForward(0.5, 13);
        robot.getCollector().positionArmExt(1000);
    }
}
