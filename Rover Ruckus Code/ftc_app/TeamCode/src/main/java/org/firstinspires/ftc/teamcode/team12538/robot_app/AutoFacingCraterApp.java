package org.firstinspires.ftc.teamcode.team12538.robot_app;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Auto (Facing Crater)", group="Linear Opmode")
public class AutoFacingCraterApp extends RoverRuckusAutoApp {
    @Override
    protected void collectMineralOffTapedAreaAndDepositToLander(MineralLocation mineralLocation) throws InterruptedException {
        robot.prepareMineralIntake();
        if(mineralLocation != MineralLocation.Unknown) {
            robot.moveForward(0.1, 15);
            sleep(500);

            robot.getCollector().disableIntake();
            robot.getCollector().flipCollectorBox(0.8);
            sleep(1000);

            if(mineralLocation == MineralLocation.Left) {
                robot.moveBackward(0.1, 20);
                robot.rotate(90, 0.2);
                robot.moveForward(0.2, 10);
                robot.rotate(45, 0.2);
            } else if(mineralLocation == MineralLocation.Right) {
                robot.moveBackward(0.1, 20);
                robot.rotate(90, 0.2);
                robot.moveForward(0.2, 40);
                robot.rotate(45, 0.2);
            } else {
                robot.moveBackward(0.1, 20);
                robot.rotate(90, 0.2);
                robot.moveForward(0.2, 25);
                robot.rotate(45, 0.2);
            }
        }
    }

    @Override
    protected void navigateToDepot(MineralLocation mineralLocation) throws InterruptedException {
        if (mineralLocation == MineralLocation.Left) {
            robot.moveForward(0.5, 70);
            robot.stop();
        } else if (mineralLocation == MineralLocation.Right) {
            robot.moveForward(0.5, 70);
            robot.stop();
        } else {
            robot.moveForward(0.5, 70);
            robot.stop();
        }

        robot.rotate(90, 0.2);
        placeTeamMarker();
    }

    @Override
    protected void navigateForParking(MineralLocation mineralLocation) {
        robot.moveBackward(0.5, 80);
    }
}
