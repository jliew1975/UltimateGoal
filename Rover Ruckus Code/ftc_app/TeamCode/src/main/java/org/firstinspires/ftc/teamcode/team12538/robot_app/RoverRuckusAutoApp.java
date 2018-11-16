package org.firstinspires.ftc.teamcode.team12538.robot_app;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.team12538.components.MineralMechanism;
import org.firstinspires.ftc.teamcode.team12538.detectors.GoldAlignDetectorExt;
import org.firstinspires.ftc.teamcode.team12538.detectors.GoldAlignDetectorExtDebug;
import org.firstinspires.ftc.teamcode.team12538.robotV1.AutoRobotV1;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

public abstract class RoverRuckusAutoApp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    public enum MineralLocation { Left, Center, Right, Unknown }

    AutoRobotV1 robot = null;
    GoldAlignDetectorExt detector = null;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            OpModeUtils.init(this);

            robot = new AutoRobotV1();
            robot.init();
            robot.init_imu();

            detector = createDetector();
            detector.enable();

            waitForStart();

            // deploy robot from lander
            // robot.unlatchFromLander();
            robot.expandMechanism();

            // move robot forward a little toward mineral
            // for gold mineral detection
            // robot.moveForward(0.5, 10);

            // locate the gold mineral location
            MineralLocation mineralLocation = locateGoldMineral();
            telemetry.addData("Mineral Location", mineralLocation);
            telemetry.addData("Mineral X-Pos", detector.getXPosition());
            telemetry.update();

            // move the gold mineral off taped area
            collectMineralOffTapedAreaAndDepositToLander(mineralLocation);

            // navigate to depot area for team marker deployment
            navigateToDepot(mineralLocation);

            // navigate to crater for parking
            navigateForParking(mineralLocation);

            sleep(5000);
            robot.stop();
        } finally {
            if(detector != null) {
                detector.disable();
            }

            OpModeUtils.stop();
        }
    }

    protected void collectMineralOffTapedAreaAndDepositToLander(MineralLocation mineralLocation) throws InterruptedException {
        robot.prepareMineralIntake();

        if(mineralLocation != MineralLocation.Unknown) {

            robot.moveForward(0.5, 12);
            robot.stop();
            robot.getCollector().autoMineralDeposit();

            if(mineralLocation == MineralLocation.Center) {
                robot.moveBackward(0.5, 10);
            } else if(mineralLocation == MineralLocation.Left) {
                robot.moveBackward(0.5, 10);
                robot.rotate(-25, 0.5);
            } else {
                robot.moveBackward(0.5, 10);
                robot.rotate(25 , 0.5);
            }

            // robot.getCollector().swingArmToDeposit(500);
            robot.getCollector().controlReleaseMineral(MineralMechanism.MineralSide.Left, 0);
            robot.getCollector().controlReleaseMineral(MineralMechanism.MineralSide.Right, 0);
            // robot.getCollector().swingArmToDeposit(0);
        }
    }

    protected void navigateToDepot(MineralLocation mineralLocation) throws InterruptedException {
        if(mineralLocation != MineralLocation.Unknown) {
            if(mineralLocation == MineralLocation.Right) {
                robot.rotate(80, 0.1);
                robot.moveForward(0.5, 28);
            } else if(mineralLocation == MineralLocation.Left) {
                robot.rotate(-25, 0.1);
                robot.moveForward(0.5, 28);
            } else {
                robot.moveForward(0.5, 40);
            }

            robot.stop();
            robot.placeTeamMarker();
        }
    }

    protected void navigateForParking(MineralLocation mineralLocation) throws InterruptedException {
        if(mineralLocation == MineralLocation.Right) {
            robot.rotate(-45, 0.1);
            robot.moveBackward(0.5, 80);
        } else if(mineralLocation == MineralLocation.Left) {
            robot.rotate(-80, 0.1);
            robot.moveBackward(0.5, 80);
        } else if(mineralLocation == MineralLocation.Center) {
            robot.rotate(-45, 0.1);
            robot.moveBackward(0.5, 80);
        } else {
            robot.rotate(45, 0.1);
            robot.moveForward(0.5, 20);
            robot.rotate(-45, 0.1);
            robot.moveBackward(0.5, 80);
        }
    }

    private MineralLocation locateGoldMineral() throws InterruptedException {
        // assumed after deployment robot is center on the middle of mineral tape
        if(detector.isFound()) {
            // TODO: need to adjust heading to align with mineral
            return MineralLocation.Center;
        }

        // if not found try left side
        robot.rotate(25, 0.2, detector);
        sleep(200);

        if(detector.isFound()) {
            return MineralLocation.Left;
        }

        robot.rotate(-60, 0.2, detector);
        sleep(200);

        if(detector.isFound()) {
            return MineralLocation.Right;
        }

        return MineralLocation.Unknown;
    }

    protected GoldAlignDetectorExt createDetector() {
        GoldAlignDetectorExt detector = new GoldAlignDetectorExtDebug();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 200; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA;
        detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        // detector.maxAreaScorer.weight = 0.005; // if using MAX_AREA scoring
        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.useDefaults();

        return detector;
    }

    protected void placeTeamMarker() {
        robot.placeTeamMarker();
    }
}
