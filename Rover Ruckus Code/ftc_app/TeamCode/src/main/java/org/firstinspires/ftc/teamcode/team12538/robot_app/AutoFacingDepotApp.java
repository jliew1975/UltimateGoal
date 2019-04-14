package org.firstinspires.ftc.teamcode.team12538.robot_app;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

@Autonomous(name="Auto (Facing Depot)", group="Linear Opmode")
public class AutoFacingDepotApp extends RoverRuckusAutoApp {
    @Override
    protected void collectMineralfromCrater(MineralLocation mineralLocation) throws InterruptedException {
        if(mineralLocation == MineralLocation.Left) {
            robot.rotate(-25, 0.3, 5.0);
        } else if(mineralLocation == MineralLocation.Right) {
            robot.rotate(15, 0.3, 5.0);
        }

        robot.getCollector().disableIntake();
    }

    @Override
    protected void depositMineral(MineralLocation mineralLocation) throws InterruptedException {
        double backwardDistance = 7;

        robot.moveBackward(0.5, backwardDistance);
        robot.getCollector().liftDepo(750, true);
        robot.getCollector().rotateDepositBox(0.67, true);
        ThreadUtils.getExecutorService().submit(new Runnable() {
            @Override
            public void run() {
                robot.getCollector().lowerDepo(true);
            }
        });

        robot.getCollector().disableIntake();
    }

    @Override
    protected void collectMineralOffTapedArea(MineralLocation mineralLocation) throws InterruptedException {
        int rotateDegree = 0;

        // robot.prepareMineralIntake();
        if(!mineralLocation.isSkipAlign()) {
            switch (mineralLocation) {
                case Center:
                    autoCollectMineral(1000, false, true);
                    break;

                case Left:
                    // robot.moveForward(0.1, 5.0);
                    autoCollectMineral(1200, false, true);
                    break;

                case Right:
                    autoCollectMineral(1200, false, true);
                    break;
            }
        }
    }

}
