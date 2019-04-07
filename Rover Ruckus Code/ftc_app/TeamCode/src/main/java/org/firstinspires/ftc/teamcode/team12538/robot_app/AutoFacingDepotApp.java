package org.firstinspires.ftc.teamcode.team12538.robot_app;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="Auto (Facing Depot)", group="Linear Opmode")
public class AutoFacingDepotApp extends RoverRuckusAutoApp {
    @Override
    protected void depositMineral(MineralLocation mineralLocation) {

    }

    @Override
    protected void collectMineralOffTapedArea(MineralLocation mineralLocation) throws InterruptedException {
        int rotateDegree = 0;

        // robot.prepareMineralIntake();
        if(!mineralLocation.isSkipAlign()) {
            switch (mineralLocation) {
                case Center:
                    robot.getCollector().autoCollectMineral(1000, false);
                    break;

                case Left:
                    robot.moveForward(0.1, 5.0);
                    robot.getCollector().autoCollectMineral(1800, false);
                    break;

                case Right:
                    robot.getCollector().autoCollectMineral(1800, false);
                    break;
            }
        }
    }

}
