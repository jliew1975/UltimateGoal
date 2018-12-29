package org.firstinspires.ftc.teamcode.team12538.robot_app;


import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverruckus.SamplingOrderDetectorExt;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team12538.detectors.GoldAlignDetectorExt;
import org.firstinspires.ftc.teamcode.team12538.detectors.GoldAlignDetectorExtDebug;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

@Autonomous(name="Auto (Facing Crater)", group="Linear Opmode")
public class AutoFacingCraterApp extends RoverRuckusAutoApp {
    @Override
    public void runOpMode() throws InterruptedException {
        super.phoneTiltPosition = 0.6;
        super.moveForwardPosition = 5.0;
        super.runOpMode();
    }

    @Override
    protected void collectMineralOffTapedArea(MineralLocation mineralLocation) throws InterruptedException {
        if(!mineralLocation.isSkipAlign()) {
            switch (mineralLocation) {
                case Center:
                    double xPos = detector.getXPosition();
                    if (!detector.isAligned()) {
                        if(xPos > 300) {
                            robot.rotate(-20, 0.1, 0.5, detector);
                        } else {
                            robot.rotate(20, 0.1, 0.5, detector);
                        }
                    }

                    break;

                case Left:
                    robot.rotate(30, 0.1, 5.0, detector);
                    break;

                case Right:
                    robot.rotate(-30, 0.1, 5.0, detector);
                    break;
            }
        }

        if(mineralLocation == MineralLocation.Center) {
            robot.moveForward(0.5, 25);
        } else {
            robot.moveForward(0.5, 30);
        }
    }

    @Override
    protected void navigateToDepot(MineralLocation mineralLocation) throws InterruptedException {
        robot.moveBackward(0.5, 15);

        if (mineralLocation == MineralLocation.Left) {
            robot.rotate(45, 0.3, 5.0);
            robot.moveForward(0.5, 35);
            robot.rotate(55, 0.3, 5.0);
            robot.strafeRight(0.5, 5.0);
            if(OpModeUtils.getTimeRemaining() >= 8) {
                // sleep for 2 seconds for other robot to move out of the way
                sleep(2000);
            }
            robot.moveForward(0.5, 30);
        } else if(mineralLocation == MineralLocation.Right) {
            robot.rotate(105, 0.3, 5.0);
            robot.moveForward(0.5, 55);
            robot.rotate(40, 0.3, 5.0);
            robot.strafeRight(0.5, 5.0);
            if(OpModeUtils.getTimeRemaining() >= 8) {
                // sleep for 2 seconds for other robot to move out of the way
                sleep(2000);
            }
            robot.moveForward(0.5, 30);
        } else {
            robot.rotate(78, 0.3, 5.0);
            robot.moveForward(0.5, 45);
            robot.rotate(40, 0.3, 5.0);
            robot.strafeRight(0.5, 5.0);
            if(OpModeUtils.getTimeRemaining() >= 8) {
                // sleep for 2 seconds for other robot to move out of the way
                sleep(2000);
            }
            robot.moveForward(0.5, 30);
        }

        placeTeamMarker();
    }

    @Override
    protected void navigateForParking(MineralLocation mineralLocation) {
        robot.moveBackward(0.5, 60);
        robot.getParkingRod().setPosition(0.5); // for breaking the crater plain to score parking points
        sleep(500);
    }

    @Override
    protected SamplingOrderDetectorExt createDetector() {
        SamplingOrderDetectorExt detector = super.createDetector();
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 200; // How far from center frame to offset this alignment zone.

        return detector;
    }
}
