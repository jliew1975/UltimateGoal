package org.firstinspires.ftc.teamcode.team12538.drive;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;

public abstract class AutoMecanumDrive extends MecanumDriveBase {


    @Override
    public void init() {
        super.init();
        MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, motors);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, motors);
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, motors);
    }




}
