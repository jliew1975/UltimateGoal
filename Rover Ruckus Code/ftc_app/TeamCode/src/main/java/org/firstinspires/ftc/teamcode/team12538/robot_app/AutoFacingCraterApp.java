package org.firstinspires.ftc.teamcode.team12538.robot_app;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

@Autonomous(name="Auto (Facing Crater)", group="Linear Opmode")
public class AutoFacingCraterApp extends RoverRuckusAutoApp {
    @Override
    public void runOpMode() throws InterruptedException {
        super.moveForwardPosition = 10.0;
        super.runOpMode();
    }

    @Override
    protected void collectMineralOffTapedArea(MineralLocation mineralLocation) throws InterruptedException {
        int rotateDegree = 0;

        if(!mineralLocation.isSkipAlign()) {
            switch (mineralLocation) {
                case Center:
                    double xPos = detector.getXPosition();
                    if (!detector.isAligned()) {
                        if(xPos > 300) {
                            robot.rotate(-20, 0.1, 0.5, detector);
                            rotateDegree = 20;
                        } else {
                            robot.rotate(20, 0.1, 0.5, detector);
                            rotateDegree = -20;
                        }
                    }

                    break;

                case Left:
                    robot.rotate(30, 0.1, 5.0, detector);
                    break;

                case Right:
                    robot.rotate(-30, 0.1, 5.0, detector);
                    robot.rotate(-20, 0.1, 0.5);
                    break;
            }
        }

        if(mineralLocation == MineralLocation.Center) {
            robot.moveBackward(0.5, 15);
        } else {
            robot.moveBackward(0.5, 25);
        }

        if(rotateDegree != 0) {
            robot.rotate(rotateDegree, 0.1, 0.5);
        }
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
