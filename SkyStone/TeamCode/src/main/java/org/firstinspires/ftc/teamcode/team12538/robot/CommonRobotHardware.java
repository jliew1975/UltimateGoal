package org.firstinspires.ftc.teamcode.team12538.robot;

import org.firstinspires.ftc.teamcode.team12538.components.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.components.OuttakeSlides;
import org.firstinspires.ftc.teamcode.team12538.components.RobotIntake;
import org.firstinspires.ftc.teamcode.team12538.components.TeleOpDrive;

/**
 * Define all robot components this class for easy maintenance
 */
public abstract class CommonRobotHardware {
    protected RobotIntake intake = new RobotIntake();
    protected OuttakeSlides outtakeSlides = new OuttakeSlides();

    public void init() {
        intake.init();
        outtakeSlides.init();
    }

    public RobotIntake getIntake() {
        return intake;
    }

    public OuttakeSlides getOuttakeSlides() {
        return outtakeSlides;
    }
}
