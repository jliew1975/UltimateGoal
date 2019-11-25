package org.firstinspires.ftc.teamcode.team12538.detectors;

public interface RobotDetectorLimit extends RobotDetector {
    double getLimit();
    void setLimit(double limit);

    double getCurrentDistance();
}
