package org.firstinspires.ftc.teamcode.team12538.robot_app;


import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team12538.detectors.GoldAlignDetectorExt;
import org.firstinspires.ftc.teamcode.team12538.detectors.GoldAlignDetectorExtDebug;

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
        robot.prepareMineralIntake();
        if(mineralLocation != MineralLocation.Unknown) {
            if(mineralLocation == MineralLocation.Center) {
                robot.moveForward(0.1, 10);
            } else {
                robot.moveForward(0.1, 15);
            }

            robot.getCollector().disableIntake();
            robot.getCollector().flipCollectorBox(0.6);
        }
    }

    @Override
    protected void navigateToDepot(MineralLocation mineralLocation) throws InterruptedException {
        if(mineralLocation != MineralLocation.Right) {
            robot.moveBackward(0.5, 3);

            if (mineralLocation == MineralLocation.Left) {
                robot.rotate(30, 0.5, 5.0);
                robot.moveForward(0.5, 20);
                robot.rotate(50, 0.5, 5.0);
                robot.strafeRight(0.3, 20);
                robot.strafeLeft(0.3,1);
                robot.moveForward(0.5, 60);
            } else {
                robot.rotate(78, 0.3, 5.0);
                robot.moveForward(0.5, 28);
                robot.rotate(40, 0.3, 5.0);
                robot.strafeRight(0.3, 20);
                robot.strafeLeft(0.3,1);
                robot.moveForward(0.5, 50);
            }

            robot.rotate(90, 0.2, 5.0);
            placeTeamMarker();
            robot.rotate(90, 0.2, 5.0);
            robot.strafeLeft(0.3, 9);
        }
    }

    @Override
    protected void navigateForParking(MineralLocation mineralLocation) {
        if(mineralLocation == MineralLocation.Right) {
            robot.moveForward(0.5, 20);
            robot.getCollector().flipCollectorBox(0d); // for touching the crater to score points
        } else {
            robot.moveForward(0.6, 60);
            robot.getCollector().flipCollectorBox(0d); // for touching the crater to score points
        }
    }

    /*
    @Override
    protected GoldAlignDetectorExt createDetector() {
        GoldAlignDetectorExt detector = new GoldAlignDetectorExtDebug();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = -60; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA;
        detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        // detector.maxAreaScorer.weight = 0.005; // if using MAX_AREA scoring
        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.useDefaults();

        return detector;
    }
    */
}
