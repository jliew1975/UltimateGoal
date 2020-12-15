package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.GlobalStorage;

@Autonomous(name="Blue (R) - 2 Wobble", group="Group 1")
public class AutoBlueTwoWobbleRightApp extends AutoBlueRightApp {
    public AutoBlueTwoWobbleRightApp() {
        super();
        GlobalStorage.wobbleCount = 2;
    }
}
