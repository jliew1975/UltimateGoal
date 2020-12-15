package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.GlobalStorage;

@Autonomous(name="Red (L) - 2 Wobble", group="Group 1")
public class AutoRedLeftTwoWobbleApp extends AutoRedLeftApp {
    public AutoRedLeftTwoWobbleApp() {
        super();
        GlobalStorage.wobbleCount = 2;
    }
}
