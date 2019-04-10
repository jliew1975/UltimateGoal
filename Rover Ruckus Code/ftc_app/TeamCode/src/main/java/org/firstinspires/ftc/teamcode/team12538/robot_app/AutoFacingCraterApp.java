package org.firstinspires.ftc.teamcode.team12538.robot_app;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

@Autonomous(name="Auto (Facing Crater)", group="Linear Opmode")
public class AutoFacingCraterApp extends RoverRuckusAutoApp {
    @Override
    protected void depositMineral(MineralLocation mineralLocation) throws InterruptedException {
        double targetAngle = 10d;

        switch(mineralLocation) {
            case Left:
                targetAngle = -18;
                break;

            case Right:
                targetAngle = -18;
                break;

            default:
                targetAngle = -18;
                break;
        }

        robot.rotate(targetAngle, 0.3, 5.0);
        robot.moveBackward(0.5, 12);
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
    protected void navigateToDepot(MineralLocation mineralLocation) throws InterruptedException {
        robot.moveForward(0.5, 10);

        if (mineralLocation == MineralLocation.Left) {
            robot.rotate(42, 1.0, 5.0);
            robot.moveForward(0.5, 35);
            robot.rotate(40, 0.5, 5.0);
            robot.strafeRight(0.5, 10.0);
            robot.moveForward(0.5, 35);
        } else if(mineralLocation == MineralLocation.Right) {
            robot.rotate(45, 1.0, 5.0);
            robot.moveForward(0.5, 35);
            robot.rotate(40, 0.5, 5.0);
            robot.strafeRight(0.5, 10.0);
            robot.moveForward(0.5, 35);
        } else {
            robot.rotate(42, 1.0, 5.0);
            robot.moveForward(0.5, 38);
            robot.rotate(40, 0.5, 5.0);
            robot.strafeRight(0.5, 10.0);
            robot.moveForward(0.5, 35);
        }

        sleep(1000);
        // robot.placeTeamMarker();
    }

    @Override
    protected void navigateForParking(MineralLocation mineralLocation) throws InterruptedException {
        // robot.strafeRight(0.5, 10.0);
        robot.moveBackward(0.5, 30);
        // robot.rotate(-10, 0.1, 0.5);
        robot.moveBackward(0.5, 35);
        robot.getParkingRod().setPosition(0d); // for breaking the crater plain to score parking points
        sleep(500);
    }
}
