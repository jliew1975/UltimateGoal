package org.firstinspires.ftc.teamcode.team12538.robot_app;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
            sleep(1000);

            // move robot forward a little toward mineral
            // for gold mineral detection
            robot.moveForward(0.1, 8);

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
        robot.moveForward(0.1, 15);
        sleep(500);

        robot.getCollector().disableIntake();
        robot.getCollector().flipCollectorBox(0.8);
        sleep(1000);

        if(mineralLocation == MineralLocation.Left) {
            robot.moveForward(0.1, 20);
        } else if(mineralLocation == MineralLocation.Right) {
            robot.moveForward(0.1, 20);
        } else {
            robot.moveForward(0.1, 10);
        }
    }

    protected void navigateToDepot(MineralLocation mineralLocation) throws InterruptedException {

        if(mineralLocation == MineralLocation.Left) {
            robot.rotate(96, 0.2);
            robot.moveBackward(0.2, 25);
        } else if(mineralLocation == MineralLocation.Right) {
            robot.rotate(80, 0.2);
            robot.moveForward(0.2, 28);
            robot.rotate(80, 0.2);
        } else {
            robot.moveForward(0.2, 18);
            robot.rotate(45, 0.2);
            robot.moveForward(0.2, 3);
            robot.rotate(80, 0.2);
        }

        robot.stop();
        robot.placeTeamMarker();
    }

    protected void navigateForParking(MineralLocation mineralLocation) throws InterruptedException {
        robot.moveForward(0.5, 80);
        robot.getCollector().flipCollectorBox(0d); // for touching the crater to score points
    }

    private MineralLocation locateGoldMineral() throws InterruptedException {
        double xPos = -1;
        if(detector.isFound()) {
            xPos = detector.getXPosition();
        }

        // assumed after deployment robot is center on the middle of mineral tape
        if(detector.isFound() && detector.isAligned()) {
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

        // not able to determine the location of gold mineral so will try
        // to use the x-position if gold mineral is found but not aligned and position is not known
        // TODO: Need logic to strafe left or right to align with gold element
        return MineralLocation.Center;
    }

    protected GoldAlignDetectorExt createDetector() {
        GoldAlignDetectorExt detector = new GoldAlignDetectorExtDebug();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        detector.alignSize = 80; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = -100; // How far from center frame to offset this alignment zone.
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
