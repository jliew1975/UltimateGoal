package org.firstinspires.ftc.teamcode.team12538.robot_app;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.team12538.robotV1.AutoRobotV1;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

public abstract class RoverRuckusAutoApp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    public enum MineralLocation { Left, Center, Right, Unknown }

    AutoRobotV1 robot = null;
    GoldAlignDetector detector = null;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            OpModeUtils.getGlobalStore().setOpMode(this);
            OpModeUtils.getGlobalStore().setHardwareMap(hardwareMap);
            OpModeUtils.getGlobalStore().setTelemetry(telemetry);

            robot = new AutoRobotV1();
            robot.init();
            robot.init_imu();

            detector = createDetector();
            detector.enable();

            waitForStart();

            // deploy robot from lander
            robot.unlatchFromLander();

            // move robot forward a little toward mineral.
            /*
            runtime.reset();
            runtime.startTime();
            robot.moveForward(0.1);
            while(OpModeUtils.getOpMode().opModeIsActive() && runtime.milliseconds() < 300) {
                idle();
            }
            robot.stop();

            sleep(1000);
            */

            MineralLocation mineralLocation = locateGoldMineral();
            telemetry.addData("Mineral Location", mineralLocation);
            telemetry.addData("Mineral X-Pos", detector.getXPosition());
            telemetry.update();

            moveMineralOffTapedArea(mineralLocation);
            /*
            moveToAllianceDepot(mineralLocation);
            navigateForParking(mineralLocation);
            */
            sleep(5000);
            robot.stop();
        } finally {
            if(detector != null) {
                detector.disable();
            }
        }
    }

    protected void moveMineralOffTapedArea(MineralLocation mineralLocation) {
        if(mineralLocation != MineralLocation.Unknown) {
            robot.moveForward(0.5, 15);
            robot.stop();
        }
    }

    protected void moveToAllianceDepot(MineralLocation mineralLocation) {
        if(mineralLocation != MineralLocation.Unknown) {
            if(mineralLocation == MineralLocation.Right) {
                robot.moveForward(0.5, 500);
            } else if(mineralLocation == MineralLocation.Left) {
                robot.moveForward(0.5, 500);
            } else {
                robot.moveForward(0.5, 500);
            }

            robot.stop();
            robot.placeTeamMarker();
        }
    }

    protected void navigateForParking(MineralLocation mineralLocation) {
        /*
        if(mineralLocation == MineralLocation.Right) {
            robot.setTargetHeading(-Math.PI/4);
            robot.moveBackwardWithEncoder(0.8, 2000);
        } else if(mineralLocation == MineralLocation.Left) {
            robot.setTargetHeading(Math.PI/2);
            robot.moveBackwardWithEncoder(0.8, 2000);
        } else if(mineralLocation == MineralLocation.Center) {
            robot.setTargetHeading(Math.PI/8);
            robot.moveBackwardWithEncoder(0.8, 2000);
        } else {
            robot.setTargetHeading(Math.PI/4);
            robot.moveBackwardWithEncoder(0.5, 500);
            robot.setTargetHeading(Math.PI/8);
            robot.moveBackwardWithEncoder(0.8, 500);
        }
        */
    }

    private MineralLocation locateGoldMineral() throws InterruptedException {
        // assumed after deployment robot is center on the middle of mineral tape
        if(detector.isFound()) {
            // TODO: need to adjust heading
            return MineralLocation.Center;
        }

        // if not found try left side
        robot.rotate(25, 0.05, detector);
        sleep(200);

        if(detector.isFound()) {
            return MineralLocation.Left;
        }

        robot.rotate(-60, 0.05, detector);
        sleep(200);

        if(detector.isFound()) {
            return MineralLocation.Right;
        }

        return MineralLocation.Unknown;
    }

    protected GoldAlignDetector createDetector() {
        GoldAlignDetector detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA;
        detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        // detector.maxAreaScorer.weight = 0.005; // if using MAX_AREA scoring
        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        return detector;
    }

    protected void placeTeamMarker() {

    }
}
