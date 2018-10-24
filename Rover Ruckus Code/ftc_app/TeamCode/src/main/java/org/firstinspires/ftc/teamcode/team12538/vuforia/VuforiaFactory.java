package org.firstinspires.ftc.teamcode.team12538.vuforia;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

import java.util.ArrayList;
import java.util.List;

public class VuforiaFactory {
    private static VuforiaFactory factory = new VuforiaFactory();

    public static VuforiaFactory getInstance() {
        return factory;
    }

    public FTCVuforia createForRoverRuckus() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        HardwareMap hardwareMap = OpModeUtils.getGlobalStore().getHardwareMap();

        int cameraMonitorViewId =
                hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //  Instantiate the Vuforia engine
        VuforiaLocalizer vuforia =
                ClassFactory.getInstance().createVuforia(VuforiaUtils.getParameter(cameraMonitorViewId));

        return new FTCVuforiaForRoverRuckus(vuforia);
    }
}
