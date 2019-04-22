package org.firstinspires.ftc.teamcode.team12538.robot_app;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

@Autonomous(name="Auto (Depot)", group="Linear Opmode")
public class AutoDepot extends RoverRuckusAutoApp {
    @Override
    protected void collectMineralfromCrater(MineralLocation mineralLocation) throws InterruptedException {
        if(mineralLocation == MineralLocation.Left) {
            robot.rotate(-27, 0.3, 5.0);
        } else if(mineralLocation == MineralLocation.Right) {
            robot.rotate(15, 0.3, 5.0);
        }

        robot.getCollector().disableIntake();
    }

    @Override
    protected void depositMineral(MineralLocation mineralLocation) throws InterruptedException {
        double backwardDistance = 6;

        robot.moveBackward(0.5, backwardDistance);
        robot.strafeRight(0.5, 3.0);
        robot.stop();

        robot.getCollector().liftDepo(750, true,true);
        robot.getCollector().flipDepoBox(true);
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
        switch (mineralLocation) {
            case Center:
                autoCollectMineral(1000, false, true);
                break;

            case Left:
                autoCollectMineral(1200, false, true);
                break;

            case Right:
                autoCollectMineral(1200, false, true);
                break;
        }
    }
}
