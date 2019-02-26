package org.firstinspires.ftc.teamcode.team12538.detectors;

import com.disnodeteam.dogecv.detectors.roverruckus.SamplingOrder;

import org.firstinspires.ftc.teamcode.team12538.robot_app.RoverRuckusAutoApp;

public interface MineralDetector {
    boolean isFound();
    boolean isAligned();

    void enable();
    void disable();
    void disableSampling();

    double getXPosition();
    SamplingOrder getCurrentOrder();
    SamplingOrder getLastOrder();
}
