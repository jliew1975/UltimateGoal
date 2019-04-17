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
                targetAngle = -12;
                break;

            case Right:
                targetAngle = -12;
                break;

            default:
                targetAngle = -12;
                backwardDistance = 13;
                break;
        }

        robot.rotate(targetAngle, 0.3, 5.0);
        robot.moveBackward(0.5, backwardDistance);
        robot.corneringRight(0.5, 15.0);
        robot.stop();
        robot.getCollector().liftDepo(750, true);
        robot.getCollector().rotateDepositBox(0.67, true);
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
            robot.corneringLeft(0.5, -40);
            robot.moveForward(0.5, 17);
            robot.corneringLeft(0.5, -26);

        } else if(mineralLocation == MineralLocation.Right) {
            robot.corneringLeft(0.5, -40);
            robot.moveForward(0.5, 17);
            robot.corneringLeft(0.5, -26);
        } else {
            robot.corneringLeft(0.5, -40);
            robot.moveForward(0.5, 17);
            robot.corneringLeft(0.5, -26);
            mineralLocation.setDistance(20d);
        }

        ThreadUtils.getExecutorService().submit(new Runnable() {
            @Override
            public void run() {
                robot.moveForward(0.5, mineralLocation.getDistance());
                robot.stop();
            }
        });

        robot.getCollector().positionArmExt(500);
        robot.placeTeamMarker();
    }

    @Override
    protected void navigateForParking(MineralLocation mineralLocation) throws InterruptedException {
        robot.moveBackward(0.5, 30);
        robot.moveBackward(0.5, 35);
        robot.stop();
        robot.getParkingRod().setPosition(0d); // for breaking the crater plain to score parking points
        sleep(100);
    }
}
