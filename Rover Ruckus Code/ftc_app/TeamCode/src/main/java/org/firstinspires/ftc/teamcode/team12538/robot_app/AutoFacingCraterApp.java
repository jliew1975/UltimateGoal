package org.firstinspires.ftc.teamcode.team12538.robot_app;


import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team12538.detectors.GoldAlignDetectorExt;
import org.firstinspires.ftc.teamcode.team12538.detectors.GoldAlignDetectorExtDebug;

@Autonomous(name="Auto (Facing Crater)", group="Linear Opmode")
public class AutoFacingCraterApp extends RoverRuckusAutoApp {
    @Override
    protected void collectMineralOffTapedAreaAndDepositToLander(MineralLocation mineralLocation) throws InterruptedException {
        robot.prepareMineralIntake();
        if(mineralLocation != MineralLocation.Unknown) {
            robot.moveForward(0.1, 10);

            robot.getCollector().disableIntake();
            robot.getCollector().flipCollectorBox(0.8);

            if(mineralLocation == MineralLocation.Left) {
                robot.moveBackward(0.5, 5);
                robot.rotate(35, 0.3, 5.0);
                robot.moveForward(0.5, 31);
                robot.rotate(45, 0.3, 5.0);
            } else if(mineralLocation == MineralLocation.Right) {
                robot.moveBackward(0.5, 5);
                robot.rotate(120, 0.2, 5.0);
                robot.moveForward(0.5, 43);
                robot.rotate(45, 0.2, 5.0);
            } else {
                robot.moveBackward(0.5, 5);
                robot.rotate(85, 0.3, 5.0);
                robot.moveForward(0.5, 46);
                robot.rotate(40, 0.3, 5.0);
            }
        }
    }

    @Override
    protected void navigateToDepot(MineralLocation mineralLocation) throws InterruptedException {
        if (mineralLocation == MineralLocation.Left) {
            robot.strafeRight(0.3, 5);
            robot.moveForward(0.5, 40);
        } else if (mineralLocation == MineralLocation.Right) {
            robot.strafeRight(0.3, 5);
            robot.moveForward(0.5, 40);
        } else {
            robot.strafeRight(0.3, 5);
            robot.moveForward(0.5, 40);
        }

        robot.rotate(90, 0.2, 5.0);
        robot.rotate(90, 0.2, 5.0);
        placeTeamMarker();
        robot.moveForward(0.5, 10);
        robot.strafeLeft(0.3, 4);
    }

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


    @Override
    protected void navigateForParking(MineralLocation mineralLocation) {
        robot.moveForward(0.6, 55);
        robot.getCollector().flipCollectorBox(0d); // for touching the crater to score points
    }
}
