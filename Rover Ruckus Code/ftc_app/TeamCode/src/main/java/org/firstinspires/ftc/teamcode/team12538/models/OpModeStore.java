package org.firstinspires.ftc.teamcode.team12538.models;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import lombok.Data;

@Data
public class OpModeStore {
    private LinearOpMode opMode = null;
    private HardwareMap hardwareMap = null;
    private Telemetry telemetry = null;

    private boolean disableLimit = false;
    private boolean disableInitPos = false;
    private boolean closeDepoArm = false;
    private boolean resetArmExtensionEncoderValue = true;

    public void clear() {
        opMode = null;
        hardwareMap = null;
        telemetry = null;
    }
}
