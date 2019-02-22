package org.firstinspires.ftc.teamcode.team12538.robot_app;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.team12538.components.MineralMechanism;

@Autonomous(name="Auto (Facing Crater - No Landing)", group="Linear Opmode")
public class AutoFacingCraterNoLandingApp extends AutoFacingCraterApp {
    @Override
    public void runOpMode() throws InterruptedException {
        super.enableLanding = false;
        super.runOpMode();
    }

    @Override
    protected void collectMineralOffTapedArea(MineralLocation mineralLocation) throws InterruptedException {
        if(mineralLocation != MineralLocation.Unknown) {
            if(mineralLocation == MineralLocation.Center) {
                robot.moveForward(0.1, 10);
            } else {
                robot.moveForward(0.1, 15);
            }
        }
    }

    @Override
    protected void navigateToDepot(MineralLocation mineralLocation) throws InterruptedException {
        // intentionally left blank for no-op
    }

    @Override
    protected void navigateForParking(MineralLocation mineralLocation) {
        robot.getCollector().flipCollectorBox(0d); // for touching the crater to score points
    }
}
