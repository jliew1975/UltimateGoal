package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red (L) - 2 Wobble", group="Group 1")
public class LeftAutoRedTwoWobbleApp extends LeftAutoRedApp {
    public LeftAutoRedTwoWobbleApp() {
        super.isPickupSecondWobbleGoal = true;
    }
}
