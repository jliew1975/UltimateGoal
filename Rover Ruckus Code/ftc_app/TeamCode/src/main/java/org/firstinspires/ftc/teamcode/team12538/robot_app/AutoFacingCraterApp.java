package org.firstinspires.ftc.teamcode.team12538.robot_app;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

@Autonomous(name="Auto (Facing Crater)", group="Linear Opmode")
public class AutoFacingCraterApp extends RoverRuckusAutoApp {
    @Override
    protected void depositMineral(MineralLocation mineralLocation) {
        robot.moveBackward(0.5, 15);
        robot.getCollector().liftDepo(750, true);
        robot.getCollector().rotateDepositBox(0.67, true);
        robot.getCollector().lowerDepo(true);
    }

    @Override
    protected void navigateToDepot(MineralLocation mineralLocation) throws InterruptedException {
        robot.moveForward(0.3, 10);

        if (mineralLocation == MineralLocation.Left) {
            robot.rotate(-105, 0.3, 5.0);
            robot.stop();
            robot.moveForward(0.5, 25);
            robot.rotate(35, 0.3, 5.0);
            robot.strafeRight(0.5, 20);
            robot.moveForward(0.5, 35);
        } else if(mineralLocation == MineralLocation.Right) {
            robot.rotate(-38, 0.3, 5.0);
            robot.moveForward(0.5, 40);
            robot.rotate(35, 0.3, 5.0);
            robot.strafeRight(0.5, 20);
            robot.moveForward(0.5, 40);
        } else {
            robot.rotate(-80, 0.3, 5.0);
            robot.moveForward(0.5, 45);
            robot.rotate(30, 0.3, 5.0);
            robot.strafeRight(0.5, 5.0);
            robot.moveForward(0.5, 30);
        }

        robot.placeTeamMarker();
    }

    @Override
    protected void navigateForParking(MineralLocation mineralLocation) throws InterruptedException {
        robot.moveBackward(0.5, 30);
        robot.rotate(-10, 0.1, 0.5);
        robot.moveBackward(0.5, 30);
        robot.getParkingRod().setPosition(0d); // for breaking the crater plain to score parking points
        sleep(500);
    }
}
