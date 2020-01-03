package org.firstinspires.ftc.teamcode.team12538.detectors;

public interface TargetPositionalDetector {
    enum Position { Left, Center, Right, Unknown }

    Position getPosition();
}
