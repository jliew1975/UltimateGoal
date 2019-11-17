package org.firstinspires.ftc.teamcode.team12538.robot;

import android.graphics.Path;

import org.firstinspires.ftc.teamcode.team12538.components.RobotFoundationClaw;
import org.firstinspires.ftc.teamcode.team12538.components.RobotOuttake;
import org.firstinspires.ftc.teamcode.team12538.components.RobotOuttakeSlides;
import org.firstinspires.ftc.teamcode.team12538.components.RobotIntake;
import org.firstinspires.ftc.teamcode.team12538.components.RobotStoneArm;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

/**
 * Define all robot components this class for easy maintenance
 */
public abstract class CommonRobotHardware {
    public RobotIntake intake = new RobotIntake();
    public RobotOuttakeSlides outtakeSlides = new RobotOuttakeSlides();
    public RobotFoundationClaw foundationClaw = new RobotFoundationClaw();
    public RobotOuttake outtake = new RobotOuttake();
    public RobotStoneArm autoStoneArm = new RobotStoneArm();

    public void init() {
        intake.init();
        outtakeSlides.init();
        foundationClaw.init();
        outtake.init();
        autoStoneArm.init();

        OpModeUtils.getGlobalStore().addComponent("foundationClaw", foundationClaw);
    }
}
