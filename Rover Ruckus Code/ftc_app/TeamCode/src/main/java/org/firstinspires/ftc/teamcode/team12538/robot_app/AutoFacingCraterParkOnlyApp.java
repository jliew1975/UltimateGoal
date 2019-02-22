package org.firstinspires.ftc.teamcode.team12538.robot_app;


import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.team12538.components.MineralMechanism;
import org.firstinspires.ftc.teamcode.team12538.detectors.GoldAlignDetectorExt;
import org.firstinspires.ftc.teamcode.team12538.detectors.GoldAlignDetectorExtDebug;

@Autonomous(name="Auto (Facing Crater - Park Only)", group="Linear Opmode")
public class AutoFacingCraterParkOnlyApp extends AutoFacingCraterApp {
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
