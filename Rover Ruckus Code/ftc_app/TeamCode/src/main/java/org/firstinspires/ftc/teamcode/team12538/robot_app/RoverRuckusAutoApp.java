package org.firstinspires.ftc.teamcode.team12538.robot_app;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.listeners.DetectorListener;
import com.disnodeteam.dogecv.detectors.roverruckus.SamplingOrder;
import com.disnodeteam.dogecv.detectors.roverruckus.SamplingOrderDetectorExt;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.team12538.robotV1.AutoRobotV1;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

public abstract class RoverRuckusAutoApp extends LinearOpMode implements DetectorListener {
    private ElapsedTime runtime = new ElapsedTime();

    public enum MineralLocation {
        Left, Center, Right, Unknown;

        private boolean skipAlign = false;

        public boolean isSkipAlign() {
            return skipAlign;
        }

        public void setSkipAlign(boolean skipAlign) {
            this.skipAlign = skipAlign;
        }
    }

    AutoRobotV1 robot = null;
    SamplingOrderDetectorExt detector = null;

    public double phoneTiltPosition = 0.74;
    public double moveForwardPosition = 4.0;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            // initialized utility so that telemetry, hardwareMap can be reference
            // without passing it to every class.
            OpModeUtils.init(this);
            OpModeUtils.getGlobalStore().setCloseDepoArm(true);
            OpModeUtils.getGlobalStore().setDisableInitPos(false);
            OpModeUtils.getGlobalStore().setResetArmExtensionEncoderValue(true);

            // Instantiate a robot class for autonomous
            robot = new AutoRobotV1();
            robot.init();
            robot.init_imu();

            detector = createDetector();
            detector.enable();

            waitForStart();

            // stamp start time for calculation of remaining seconds in autonomous
            OpModeUtils.stampStartTime();

            // disable sampling logic
            detector.disableSampling();

            while(opModeIsActive()) {
                // deploy robot from lander
                robot.unlatchFromLander();

                // expand mineral machenism for mineral detection and collection
                // robot.expandMechanism();

                // move robot forward a little toward mineral
                // for gold mineral detection
                robot.moveForward(0.1, moveForwardPosition);

                // locate the gold mineral location
                MineralLocation mineralLocation = locateGoldMineral();
                telemetry.addData("Mineral Location", mineralLocation);
                telemetry.addData("Mineral X-Pos", detector.getXPosition());
                telemetry.addData("phoneTiltPosition", robot.getPhoneTilt().getPosition());
                telemetry.update();

                // move the gold mineral off taped area
                collectMineralOffTapedArea(mineralLocation);

                // navigate to depot area for team marker deployment
                navigateToDepot(mineralLocation);

                // navigate to crater for parking
                navigateForParking(mineralLocation);

                robot.stop();
                break;
            }
        } finally {
            if(detector != null) {
                detector.disable();
            }

            OpModeUtils.stop();
        }
    }

    @Override
    public void onEvent() {
        telemetry.addData("Sampling Result", detector.getLastOrder());
        telemetry.update();
    }

    protected void collectMineralOffTapedArea(MineralLocation mineralLocation) throws InterruptedException {
        // robot.prepareMineralIntake();
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

        robot.moveForward(0.5, 30);
        // robot.getCollector().disableIntake();
        // robot.getCollector().flipCollectorBox(0.6);
    }

    protected void navigateToDepot(MineralLocation mineralLocation) throws InterruptedException {
        if(mineralLocation == MineralLocation.Left) {
            robot.moveForward(0.5, 10);
            robot.rotate(-50, 0.5, 5.0);
            robot.moveForward(0.5, 10);
            robot.placeTeamMarker();
            robot.strafeLeft(0.5, 5.0);
        } else if(mineralLocation == MineralLocation.Right) {
            robot.moveForward(0.2, 17);
            robot.rotate(70, 0.5, 5.0);
            robot.moveForward(0.5, 10);
            robot.placeTeamMarker();
            robot.moveBackward(0.5, 10);
            robot.rotate(30, 0.5, 5.0);
            robot.moveForward(0.5, 45);
            robot.rotate(30, 0.5, 5.0);
            robot.strafeRight(0.5, 10.0);
        } else {
            robot.moveForward(0.2, 10);
            robot.placeTeamMarker();
            robot.rotate(45, 0.5, 5.0);
            robot.moveForward(0.5, 20);
            robot.moveBackward(0.5, 5);
            robot.rotate(50, 0.5, 5.0);
            robot.strafeRight(0.5, 5.0);
        }
    }

    protected void navigateForParking(MineralLocation mineralLocation) throws InterruptedException {
        if(mineralLocation != MineralLocation.Left) {
            robot.moveForward(0.5, 50);
            robot.getCollector().flipCollectorBox(1d); // for breaking the crater plain to score parking points
        } else {
            robot.moveBackward(0.5, 50);
            robot.getParkingRod().setPosition(0.5); // for breaking the crater plain to score parking points
        }

        sleep(500);
    }

    private MineralLocation locateGoldMineral() throws InterruptedException {
        double xPos = -1;

        SamplingOrder samplingOrder = detector.getLastOrder();

        if(samplingOrder == SamplingOrder.UNKNOWN) {
            samplingOrder = detector.getCurrentOrder();
        }

        MineralLocation location = MineralLocation.Unknown;

        switch(samplingOrder) {
            case CENTER:
                location = MineralLocation.Center;
                break;

            case LEFT:
                location = MineralLocation.Left;
                break;

            case RIGHT:
                location = MineralLocation.Right;
                break;

            default:
                robot.rotate(30, 0.1, 5.0);
                double angleRotated = Math.abs(robot.rotate(-60, 0.1, 5.0, detector));
                if(angleRotated < 25) {
                    location = MineralLocation.Left;
                } else if(angleRotated > 40) {
                    location = MineralLocation.Right;
                } else {
                    location = MineralLocation.Center;
                }

                location.setSkipAlign(true);
        }

        return location;
    }

    protected SamplingOrderDetectorExt createDetector() {
        SamplingOrderDetectorExt detector = new SamplingOrderDetectorExt();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 150; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames
        // detector.areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA;
        // detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
        detector.maxAreaScorer.weight = 0.001; // if using MAX_AREA scoring
        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.useDefaults();
        detector.listener = this;
        return detector;
    }

    protected void placeTeamMarker() {
        robot.placeTeamMarker();
    }
}
