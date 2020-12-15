package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.GlobalStorage;

@Autonomous(name="Red (R) - 2 Wobble", group="Group 1")
public class AutoRedRightTwoWobbleApp extends AutoRedRightApp {
    public AutoRedRightTwoWobbleApp() {
        super();
        GlobalStorage.wobbleCount = 2;
    }
}
