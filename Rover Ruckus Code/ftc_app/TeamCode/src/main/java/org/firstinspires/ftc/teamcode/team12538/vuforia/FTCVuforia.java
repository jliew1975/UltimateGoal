package org.firstinspires.ftc.teamcode.team12538.vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

public abstract class FTCVuforia {
    protected VuforiaLocalizer vuforia;

    protected VuforiaTrackables targetsRoverRuckusOT = null;

    // For convenience, gather together all the trackable objects in one easily-iterable collection */
    protected List<VuforiaTrackable> allTrackables = new ArrayList<>();

    public FTCVuforia(VuforiaLocalizer vuforia) {
        this.vuforia = vuforia;
    }

    public abstract void loadTrackable(String assetName);
}
