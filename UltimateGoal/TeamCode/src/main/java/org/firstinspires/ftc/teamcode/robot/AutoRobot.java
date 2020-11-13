package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.components.CommonComponents;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.OpModeUtils;

import lombok.Data;

@Data
public class AutoRobot extends CommonComponents {
    /**
     * Road Runner Drive
     */
    protected SampleMecanumDrive drive;

    public void init() {
        super.init();

        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();
        drive = new SampleMecanumDrive(hardwareMap);
    }
}
