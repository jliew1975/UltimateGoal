package org.firstinspires.ftc.teamcode.team12538.robot_app;


import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.team12538.detectors.EnhancedMineralOrderDetector;
import org.firstinspires.ftc.teamcode.team12538.detectors.SamplingOrder;
import org.firstinspires.ftc.teamcode.team12538.robotV1.AutoRobotV1;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

@Autonomous(name="Auto (Crater) - TM First", group="Linear Opmode")
public class AutoCraterTeamMarkerFirst extends RoverRuckusAutoApp {

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
                robot.getRobotLatch().powerLiftRunToPosition(1.0, 3730);
                robot.moveForward(0.3, 3.0);
                ThreadUtils.getExecutorService().submit(new Runnable() {
                    @Override
                    public void run() {
                        robot.getRobotLatch().powerLiftRunToPosition(1.0, 0);
                    }
                });
            }

            double startAngle = robot.getAngle();

            telemetry.addData("startAngle", startAngle);
            telemetry.update();

            double currAngle = robot.rotate(-90, 0.5,  5.0);
            robot.corneringRight(0.5, 15); // align with landers
            sleep(800);

            telemetry.addData("Sampling Result", detector.getLastOrder());
            telemetry.update();

            detector.disableSampling();

            robot.getCameraTilt().setPosition(0.94);
            navigateToDepot();

            SamplingOrder order = detector.getLastOrder();
            navigateBackToLander(order);

            /*

            robot.rotate(-20, 0.5, 5.0);
            MineralLocation mineralLocation = locateGoldMineral();

            collectMineralOffTapedArea(mineralLocation);
            collectMineralfromCrater(mineralLocation);
            depositMineral(mineralLocation);

            navigateForParking(mineralLocation);
            */
            robot.stop();
        } finally {
            if(detector != null) {
                detector.disable();
            }

            OpModeUtils.stop();
        }
    }

    protected void navigateToDepot() throws InterruptedException {
        robot.corneringLeft(0.5, -40);
        robot.moveForward(0.5, 20);
        robot.corneringLeft(0.5, -26);

        ThreadUtils.getExecutorService().submit(new Runnable() {
            @Override
            public void run() {
                robot.moveForward(0.5, 25);
            }
        });

        robot.getCollector().positionArmExt(500);
        robot.placeTeamMarker();
    }

    protected void navigateBackToLander(SamplingOrder order) throws InterruptedException {
        robot.moveBackward(0.5, 23.0);
        robot.corneringLeft(0.5, 26);
        robot.moveBackward(0.5, 40);

        if(order == SamplingOrder.RIGHT) {
            robot.rotate(-120, 0.5, 5.0);
            robot.moveBackward(0.3, 3.0);
            locateGoldMineral();
            collectMineralOffTapedArea(MineralLocation.Right);
            robot.getCollector().disableIntake();
        }
    }

    protected MineralLocation locateGoldMineral() throws InterruptedException {
        MineralLocation location = MineralLocation.Unknown;

        double rotAngle = robot.rotate(50, 0.1, -1, detector);

        if(!detector.isAligned()) {
            robot.rotate(-50, 0.05, -1, detector);
        }

        return location;
    }

    @Override
    protected void depositMineral(MineralLocation mineralLocation) throws InterruptedException {
        double targetAngle = 10d;
        double backwardDistance = 13;

        switch(mineralLocation) {
            case Left:
                targetAngle = -12;
                break;

            case Right:
                targetAngle = -12;
                break;

            default:
                targetAngle = -12;
                backwardDistance = 13;
                break;
        }

        robot.rotate(targetAngle, 0.3, 5.0);
        robot.moveBackward(0.5, backwardDistance);
        robot.corneringRight(0.5, 15.0);
        robot.getCollector().liftDepo(750, true);
        robot.getCollector().rotateDepositBox(0.67, true);
        ThreadUtils.getExecutorService().submit(new Runnable() {
            @Override
            public void run() {
                robot.getCollector().lowerDepo(true);
            }
        });
    }

    @Override
    protected void navigateForParking(MineralLocation mineralLocation) throws InterruptedException {
        robot.moveForward(0.3, 10);
        robot.getCollector().positionArmExt(1000);
    }

    @Override
    protected EnhancedMineralOrderDetector createDetector() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        EnhancedMineralOrderDetector detector = new EnhancedMineralOrderDetector();
        detector.VUFORIA_KEY = "AWbfTmn/////AAABmY0xuIe3C0RHvL3XuzRxyEmOT2OekXBSbqN2jot1si3OGBObwWadfitJR/D6Vk8VEBiW0HG2Q8UAEd0//OliF9aWCRmyDJ1mMqKCJZxpZemfT5ELFuWnJIZWUkKyjQfDNe2RIaAh0ermSxF4Bq77IDFirgggdYJoRIyi2Ys7Gl9lD/tSonV8OnldIN/Ove4/MtEBJTKHqjUEjC5U2khV+26AqkeqbxhFTNiIMl0LcmSSfugGhmWFGFtuPtp/+flPBRGoBO+tSl9P2sV4mSUBE/WrpHqB0Jd/tAmeNvbtgQXtZEGYc/9NszwRLVNl9k13vrBcgsiNxs2UY5xAvA4Wb6LN7Yu+tChwc+qBiVKAQe09\n";
        detector.init(hardwareMap.appContext,CameraViewDisplay.getInstance(), DogeCV.CameraMode.WEBCAM, true, webcamName);

        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = -120; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames
        // detector.areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA;
        // detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
        detector.maxAreaScorer.weight = 0.001; // if using MAX_AREA scoring
        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.yMinOffset = -80;
        detector.yMaxOffset = 110;

        detector.useDefaults();
        detector.listener = this;
        return detector;
    }
}
