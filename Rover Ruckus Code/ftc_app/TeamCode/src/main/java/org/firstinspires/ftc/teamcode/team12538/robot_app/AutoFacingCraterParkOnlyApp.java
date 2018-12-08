package org.firstinspires.ftc.teamcode.team12538.robot_app;


import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team12538.detectors.GoldAlignDetectorExt;
import org.firstinspires.ftc.teamcode.team12538.detectors.GoldAlignDetectorExtDebug;

@Autonomous(name="Auto (Facing Crater - Park Only)", group="Linear Opmode")
public class AutoFacingCraterParkOnlyApp extends RoverRuckusAutoApp {
    @Override
    public void runOpMode() throws InterruptedException {
        super.phoneTiltPosition = 0.22;
        super.moveForwardPosition = 5.0;
        super.runOpMode();
    }

    @Override
    protected void collectMineralOffTapedArea(MineralLocation mineralLocation) throws InterruptedException {
        robot.prepareMineralIntake();
        if(mineralLocation != MineralLocation.Unknown) {
            if(mineralLocation == MineralLocation.Center) {
                robot.moveForward(0.1, 10);
            } else {
                robot.moveForward(0.1, 15);
            }

            robot.getCollector().disableIntake();
            robot.getCollector().flipCollectorBox(0.6);
        }
    }

    @Override
    protected void navigateToDepot(MineralLocation mineralLocation) throws InterruptedException {
        // intentionally left blank for no-op
    }

    @Override
    protected void navigateForParking(MineralLocation mineralLocation) {
        robot.moveForward(0.5, 20.0);
        robot.getCollector().flipCollectorBox(0d); // for touching the crater to score points
    }
}
