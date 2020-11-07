package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.components.CommonComponents;
import org.firstinspires.ftc.teamcode.components.Shooter;
import org.firstinspires.ftc.teamcode.drive.teleop.MecanumDrive;

public class TeleOpRobot extends CommonComponents {
    private MecanumDrive drive = new MecanumDrive();

    public void init() {
        super.init();

        drive.init();
    }

    /**
     * Control for player 1
     * @param gamepad
     */
    public void controlA(Gamepad gamepad) {
        drive.navigateWithGamepad(gamepad);

        // robot components controls
        super.get(Shooter.class).control(gamepad);
    }

    /**
     * Control for player 2
     * @param gamepad
     */
    public void controlB(Gamepad gamepad) {

    }
}
