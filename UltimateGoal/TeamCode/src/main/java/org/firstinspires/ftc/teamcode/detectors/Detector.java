package org.firstinspires.ftc.teamcode.detectors;

import org.firstinspires.ftc.teamcode.detectors.enums.RingCount;

public interface Detector {
    void init() throws InterruptedException;
    void activate();
    void deactivate();
    RingCount getRingCount();
}
