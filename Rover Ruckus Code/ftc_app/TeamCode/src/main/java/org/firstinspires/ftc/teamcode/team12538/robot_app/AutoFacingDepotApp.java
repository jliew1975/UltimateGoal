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
                    autoCollectMineral(1500, false, true);
                    break;

                case Left:
                    robot.moveForward(0.1, 5.0);
                    autoCollectMineral(1500, false, true);
                    break;

                case Right:
                    autoCollectMineral(1500, false, true);
                    break;
            }
        }
    }

}
