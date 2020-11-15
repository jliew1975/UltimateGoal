package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.components.CommonComponents;
import org.firstinspires.ftc.teamcode.components.WobbleArm;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.OpModeUtils;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

import lombok.Data;

@Data
public class AutoRobot extends CommonComponents {
    /**
     * Road Runner Drive
     */
    protected SampleMecanumDrive drive;

    /**
     * Using this boolean eliminate the need for sleep
     */
    protected volatile boolean isBusy = false;

    public void init() {
        super.init();

        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();
        drive = new SampleMecanumDrive(hardwareMap);
    }

    public void prepareWobbleArm() {
        WobbleArm wobbleArm = get(WobbleArm.class);
        ThreadUtils.getExecutorService().submit(()-> {
            wobbleArm.runToPosition(770);
        });
    }

    public void dropWobbleGoal() {
        WobbleArm wobbleArm = get(WobbleArm.class);

        isBusy = true;

        ThreadUtils.getExecutorService().submit(()-> {
            try {
                wobbleArm.runToPosition(770);
                wobbleArm.unlatch();
                ThreadUtils.sleep(500);
                wobbleArm.runToPosition(0);
            } finally {
                isBusy = false;
            }
        });

        while (isBusy) {
            ThreadUtils.idle();
        }

        // ThreadUtils.sleep(3000);
    }
}
