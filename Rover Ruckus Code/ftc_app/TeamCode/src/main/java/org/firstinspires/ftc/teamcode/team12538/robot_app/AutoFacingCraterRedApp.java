package org.firstinspires.ftc.teamcode.team12538.robot_app;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto Red (Facing Crater)", group="Linear Opmode")
public class AutoFacingCraterRedApp extends RoverRuckusAutoApp {

    @Override
    protected void navigateToDepot(MineralLocation mineralLocation) {
        if(mineralLocation != MineralLocation.Unknown) {
            robot.moveBackward(0.3, 100);
            robot.stop();
        }

        // robot.setTargetHeading(-Math.PI / 4);
        robot.stop();

        if (mineralLocation == MineralLocation.Left) {
            robot.moveForward(0.5, 800);
            robot.stop();
        } else if (mineralLocation == MineralLocation.Center) {
            robot.moveForward(0.5, 1500);
            robot.stop();
        } else {
            robot.moveForward(0.5, 2000);
            robot.stop();
        }

        // robot.setTargetHeading(-Math.PI/8);

        robot.moveForward(0.5, 1000);
        placeTeamMarker();
    }

    @Override
    protected void navigateForParking(MineralLocation mineralLocation) {
        robot.moveBackward(0.3, 500);
    }
}
