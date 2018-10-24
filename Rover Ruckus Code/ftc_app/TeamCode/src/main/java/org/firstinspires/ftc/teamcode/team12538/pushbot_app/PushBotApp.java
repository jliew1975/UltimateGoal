package org.firstinspires.ftc.teamcode.team12538.pushbot_app;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.team12538.pushbot.PushBot;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

@Autonomous(name = "Pushbot Auto", group = "Test")
@Disabled
public class PushBotApp extends LinearOpMode {
    GoldAlignDetector detector;

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtils.getGlobalStore().setHardwareMap(hardwareMap);
        OpModeUtils.getGlobalStore().setTelemetry(telemetry);

        PushBot robot = new PushBot();
        robot.init();

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

        waitForStart();

        doTelemetry();

        sleep(2000);

        /*
        while(opModeIsActive() && !detector.isFound()) {
            robot.pivotLeft();
        }
        */

        // stop the robot so it doesn't overshoot
        robot.stop();

        doTelemetry();

        // get the xPos from detector to determine if we need to adjust angle
        double xPos = detector.getXPosition();
        if (xPos < 270) {
            double curXPos = xPos;
            while(curXPos < 270) {
                robot.pivotLeft();
                curXPos = detector.getXPosition();
            }
        } else if (xPos > 370) {
            double curXPos = xPos;
            while(xPos > 370) {
                robot.pivotRight();
                curXPos = detector.getXPosition();
            }
        }

        robot.stop();

        doTelemetry();

        robot.goForward();
        sleep(2000);
        robot.stop();

        if(xPos < 270) { // left

        } else if(xPos > 370) { // right

        } else { // center

        }
    }

    private void doTelemetry() {
        telemetry.addData("IsFound", detector.isFound());
        telemetry.addData("IsAligned", detector.isAligned());
        telemetry.addData("X-Pos", detector.getXPosition());
        telemetry.update();
    }
}
