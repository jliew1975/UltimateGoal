package org.firstinspires.ftc.teamcode.team12538.robot_app;

import android.graphics.Path;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.team12538.components.MineralMechanism;
import org.firstinspires.ftc.teamcode.team12538.detectors.DetectorListener;
import org.firstinspires.ftc.teamcode.team12538.detectors.EnhancedMineralOrderDetector;
import org.firstinspires.ftc.teamcode.team12538.detectors.MineralDetector;
import org.firstinspires.ftc.teamcode.team12538.robotV1.AutoRobotV1;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

public abstract class RoverRuckusAutoApp extends LinearOpMode implements DetectorListener {
    private ElapsedTime runtime = new ElapsedTime();

    public enum MineralLocation {
        Left, Center, Right, Unknown;

        private double distance = 20d;

        public double getDistance() {
            return distance;
        }

        public void setDistance(double distance) {
            this.distance = distance;
        }
    }

    AutoRobotV1 robot = null;
    MineralDetector detector = null;

    public boolean isMultiMineral = false;
    public boolean enableLanding = true;

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

            if(isStopRequested()) {
                return;
            }

            // stamp start time for calculation of remaining seconds in autonomous
            OpModeUtils.stampStartTime();

            // deploy robot from lander
            if(enableLanding) {
                robot.getRobotLatch().powerLiftRunToPosition(1.0, 7500);
                robot.moveForward(0.3, 3.0);
                /*
                ThreadUtils.getExecutorService().submit(new Runnable() {
                        @Override
                        public void run() {
                            robot.getRobotLatch().powerLiftRunToPosition(1.0, 0);
                        }
                });
                */
            }

            double startAngle = robot.getAngle();

            telemetry.addData("startAngle", startAngle);
            telemetry.update();

            // rotate robot and move a little
            // for gold mineral detection
            double currAngle = robot.rotate(-110, 0.5,  5.0);
            robot.moveForward(0.3, 5.0);

            MineralLocation mineralLocation = locateGoldMineral();
            collectMineralOffTapedArea(mineralLocation);

            collectMineralfromCrater(mineralLocation);
            depositMineral(mineralLocation);

            navigateToDepot(mineralLocation);
            navigateForParking(mineralLocation);

            robot.stop();
        } finally {
            if(detector != null) {
                detector.disable();
            }

            OpModeUtils.stop();
        }
    }

    protected MineralLocation locateGoldMineral() throws InterruptedException {
        MineralLocation location = MineralLocation.Unknown;

        double rotAngle = robot.rotate(120, 0.1, -1, detector);

        if(rotAngle < 40) {
            location = MineralLocation.Right;
        } else if(rotAngle > 40 && rotAngle < 70) {
            location = MineralLocation.Center;
        } else {
            location = MineralLocation.Left;
        }

        telemetry.addData("location", location);
        telemetry.addData("IsAlign", detector.isAligned());
        telemetry.update();

        if(!detector.isAligned()) {
            robot.rotate(-60, 0.05, -1, detector);
        }

        return location;
    }

    protected void collectMineralOffTapedArea(MineralLocation mineralLocation) throws InterruptedException {
        collectMineralOffTapedArea(mineralLocation, 0);
    }

    protected void collectMineralOffTapedArea(MineralLocation mineralLocation, int offset) throws InterruptedException {
        int rotateDegree = 0;

        switch (mineralLocation) {
            case Center:
                autoCollectMineral(800 + offset, false, true);
                break;

            case Left:
                autoCollectMineral(1300 + offset, false, true);
                break;

            case Right:
                autoCollectMineral(1300 + offset, false, true);
                break;
        }
    }

    protected void collectMineralfromCrater(MineralLocation mineralLocation) throws InterruptedException {
        if(mineralLocation == MineralLocation.Left) {
            robot.rotate(-20, 0.3, 5.0);
        } else if(mineralLocation == MineralLocation.Right) {
            robot.rotate(15, 0.3, 5.0);
        }

        robot.stop();
        robot.moveForward(0.3, 6.5);
        robot.stop();

        if(isMultiMineral) {
            switch (mineralLocation) {
                case Center:
                    autoCollectMineral(800, true, !isMultiMineral);
                    break;

                case Left:
                    autoCollectMineral(500, true, !isMultiMineral);
                    break;

                case Right:
                    autoCollectMineral(500, true, !isMultiMineral);
                    break;
            }
        }

        robot.getCollector().disableIntake();
    }

    protected abstract void depositMineral(MineralLocation mineralLocation) throws InterruptedException;

    protected void autoCollectMineral(int targetPosition, boolean isPrepFirst, boolean isTransfer) {
        final MineralMechanism collector = robot.getCollector();

        if(isPrepFirst) {
            collector.flipCollectorBox(collector.intakeFlipPrepPos);
            collector.positionArmExt(1000);
        }

        collector.flipCollectorBox(collector.intakeFlipDownPos);
        collector.enableIntake(MineralMechanism.Direction.InTake);
        collector.positionArmExt(targetPosition);
        collector.autoMineralDeposit(true, isTransfer);
    }

    protected void navigateToDepot(MineralLocation mineralLocation) throws InterruptedException {
        if(mineralLocation == MineralLocation.Left) {
            robot.corneringLeft(0.5, -40);
            robot.moveForward(0.5, 5.5);
            robot.corneringRight(0.5, -61);
        } else if(mineralLocation == MineralLocation.Right) {
            robot.corneringLeft(0.5, -40);
            robot.moveForward(0.5, 5.5);
            robot.corneringRight(0.5, -61);
        } else {
            robot.corneringLeft(0.5, -40);
            robot.moveForward(0.5, 5.5);
            robot.corneringRight(0.5, -62);
        }

        ThreadUtils.getExecutorService().submit(new Runnable() {
            @Override
            public void run() {
                robot.moveForward(0.5, 5);
            }
        });

        robot.getCollector().positionArmExt(1000);
        robot.placeTeamMarker();
    }

    protected void navigateForParking(MineralLocation mineralLocation) throws InterruptedException {
        robot.moveBackward(0.5, 30);
        robot.strafeLeft(0.5, 20);
        robot.moveBackward(0.5, 35);
        robot.getParkingRod().setPosition(0d); // for breaking the crater plain to score parking points
        sleep(100);
    }

    protected EnhancedMineralOrderDetector createDetector() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        EnhancedMineralOrderDetector detector = new EnhancedMineralOrderDetector();
        detector.VUFORIA_KEY = "AWbfTmn/////AAABmY0xuIe3C0RHvL3XuzRxyEmOT2OekXBSbqN2jot1si3OGBObwWadfitJR/D6Vk8VEBiW0HG2Q8UAEd0//OliF9aWCRmyDJ1mMqKCJZxpZemfT5ELFuWnJIZWUkKyjQfDNe2RIaAh0ermSxF4Bq77IDFirgggdYJoRIyi2Ys7Gl9lD/tSonV8OnldIN/Ove4/MtEBJTKHqjUEjC5U2khV+26AqkeqbxhFTNiIMl0LcmSSfugGhmWFGFtuPtp/+flPBRGoBO+tSl9P2sV4mSUBE/WrpHqB0Jd/tAmeNvbtgQXtZEGYc/9NszwRLVNl9k13vrBcgsiNxs2UY5xAvA4Wb6LN7Yu+tChwc+qBiVKAQe09\n";
        detector.init(hardwareMap.appContext,CameraViewDisplay.getInstance(), DogeCV.CameraMode.WEBCAM, true, webcamName);

        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = -140; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames
        // detector.areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA;
        // detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
        detector.maxAreaScorer.weight = 0.001; // if using MAX_AREA scoring
        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.yMinOffset = -180;
        detector.yMaxOffset = 110;

        detector.useDefaults();
        detector.listener = this;
        return detector;
    }

    @Override
    public void onEvent() {
        telemetry.addData("Sampling Result", detector.getLastOrder());
        telemetry.update();
    }

    protected void hold() {
        while(opModeIsActive()) {

        }
    }

}
