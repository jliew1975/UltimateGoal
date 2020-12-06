package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public interface Robot {
    Pose2d getCurrentPoss();

    SampleMecanumDrive getDrive();
}
