package org.firstinspires.ftc.teamcode.team12538.detectors;

import com.disnodeteam.dogecv.DogeCV;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public interface MineralDetector {
    boolean isFound();
    boolean isAligned();

    void enable();
    void enableVuMarkDetection();
    void disable();
    void disableSampling();

    double getXPosition();
    SamplingOrder getCurrentOrder();
    SamplingOrder getLastOrder();

    DogeCV.VuMark findVuMark();
    Boolean isVuMarkVisible();
    Orientation getRobotOrientation();
    VectorF getRobotTranslation();

    void setCAMERA_FORWARD_DISPLACEMENT(int forwardDisplacement);
    void setCAMERA_LEFT_DISPLACEMENT(int leftDisplacement);
    void setCAMERA_VERTICAL_DISPLACEMENT(int verticalDisplacement);
    
}
