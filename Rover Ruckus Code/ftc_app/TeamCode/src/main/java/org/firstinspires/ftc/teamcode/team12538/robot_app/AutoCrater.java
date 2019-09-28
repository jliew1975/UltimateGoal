package org.firstinspires.ftc.teamcode.team12538.robot_app;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

@Autonomous(name="Auto (Crater)", group="Linear Opmode")
public class AutoCrater extends RoverRuckusAutoApp {
    @Override
    protected void depositMineral(MineralLocation mineralLocation) throws InterruptedException {
        double targetAngle = 10d;
        double backwardDistance = 13;

        switch(mineralLocation) {
            case Left:
                targetAngle = -11;
                break;

            case Right:
                targetAngle = -1;
                break;

            default:
                targetAngle = -11;
                break;
        }

        robot.rotate(targetAngle, 0.3, 5.0);
        robot.moveBackward(0.5, backwardDistance);
        robot.corneringRight(0.5, 15.0);
        robot.stop();
        robot.getCollector().liftDepo(750, true,false);
        robot.getCollector().flipDepoBox(true);
        ThreadUtils.getExecutorService().submit(new Runnable() {
            @Override
            public void run() {
                robot.getCollector().lowerDepo(true);
            }
        });
    }

    @Override
    protected void navigateToDepot(final MineralLocation mineralLocation) throws InterruptedException {
        if (mineralLocation == MineralLocation.Left) {
            robot.corneringLeft(0.5, -41);
            robot.moveForward(0.5, 18);
            robot.corneringLeft(0.5, -25);

        } else if(mineralLocation == MineralLocation.Right) {
            robot.corneringLeft(0.5, -41);
            robot.moveForward(0.5, 19);
            robot.corneringLeft(0.5, -25);
        } else {
            robot.corneringLeft(0.5, -41);
            robot.moveForward(0.5, 17);
            robot.corneringLeft(0.5, -25);
        }

        /*
        ThreadUtils.getExecutorService().submit(new Runnable() {
            @Override
            public void run() {
                robot.moveForward(0.5, mineralLocation.getDistance());
            }
        });
        */

        robot.getCollector().positionArmExt(1000);
        robot.placeTeamMarker();
    }

    @Override
    protected void navigateForParking(MineralLocation mineralLocation) throws InterruptedException {
        robot.moveBackward(0.5, 30);
        robot.strafeRight(0.5, 5);
        robot.moveBackward(0.5, 35);
        // robot.getParkingRod().setPosition(0d); // for breaking the crater plain to score parking points
        sleep(100);
    }
}
