package org.firstinspires.ftc.teamcode.team12538.vuforia;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * Created by jliew1975 on 1/23/18.
 */

public class VuMarkReader {
    enum RunMode { READ, STOP }

    RunMode runMode = RunMode.READ;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    VuforiaTrackables relicTrackables = null;
    VuforiaTrackable relicTemplate = null;

    VuforiaListener listener;
    ExecutorService execService = Executors.newFixedThreadPool(1);

    public VuMarkReader(VuforiaListener listener) {
        this.listener = listener;
    }

    public void init(HardwareMap hardwareMap) {
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AWVGefb/////AAAAGedlPmn1ZUEHqXFtAqsWMjgAUJIwIAU81tlTWgCANApawYPJnKlyulGxzZNv6luzr8M7+Ku74b/i3TL+zmaSGOO6IJYoKKKd1eCbbcZyJm31m1i7PfZu9+MREAOqPV3kf6shs2eZnw9SCLpoa4t2PKfCSRla2cP7gfPayKfZvyNHi3JLrbYcE/nTQr2+TLO++ndG3y9dl3yQ1Ca9Sj5ijTb+Q9JsCi0uoAcdNQtwODqcvUR2t/DTzFcjPKJJridzPLI73v/xFJMrQCY+J5kpcsgTiif9mqvQKSOgxBuZr8+MxeRBn0GYAXuWk7FW9+QYpPydhsynIyuNVpWwmTBVh2MdP5vvtv1hLO2buPcm9wi9";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
    }

    public void activate() {
        relicTrackables.activate();
        execService.submit(new Runnable() {
            @Override
            public void run() {
                while(getRunMode() == RunMode.READ) {
                    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                    if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                        listener.setVuMark(vuMark);
                    }
                }
            }
        });
    }

    public void deactivate() {
        this.runMode = RunMode.STOP;
        execService.shutdown();
    }

    private RunMode getRunMode() {
        return runMode;
    }
}
