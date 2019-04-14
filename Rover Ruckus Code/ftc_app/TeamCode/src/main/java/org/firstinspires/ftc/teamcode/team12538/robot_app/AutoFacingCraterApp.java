package org.firstinspires.ftc.teamcode.team12538.robot_app;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

@Autonomous(name="Auto (Facing Crater)", group="Linear Opmode")
public class AutoFacingCraterApp extends RoverRuckusAutoApp {
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
        // robot.moveForward(0.5, 12);

        if (mineralLocation == MineralLocation.Left) {
            robot.corneringLeft(0.5, -40);
            // robot.rotate(42, 1.0, 5.0);
            robot.moveForward(0.5, 20);
            robot.corneringLeft(0.5, -20);
            // robot.rotate(40, 0.5, 5.0);
            // robot.strafeRight(0.5, 10.0);
            robot.moveForward(0.5, 25);
        } else if(mineralLocation == MineralLocation.Right) {
            robot.corneringLeft(0.5, -40);
            // robot.rotate(45, 1.0, 5.0);
            robot.moveForward(0.5, 19);
            robot.corneringLeft(0.5, -21);
            // robot.rotate(40, 0.5, 5.0);
            // robot.strafeRight(0.5, 10.0);
            robot.moveForward(0.5, 25);
        } else {
            // robot.rotate(42, 1.0, 5.0);
            robot.corneringLeft(0.5, -40);
            robot.moveForward(0.5, 23);
            robot.corneringLeft(0.5, -20);
            // robot.rotate(40, 0.5, 5.0);
            // robot.strafeRight(0.5, 10.0);
            robot.moveForward(0.5, 20);
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
