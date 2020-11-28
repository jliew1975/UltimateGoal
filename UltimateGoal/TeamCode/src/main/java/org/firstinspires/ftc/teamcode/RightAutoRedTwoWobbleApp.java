package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red (R) - 2 Wobble", group="Group 1")
public class RightAutoRedTwoWobbleApp extends RightAutoRedApp {
    public RightAutoRedTwoWobbleApp() {
        super.isPickupSecondWobbleGoal = true;
    }
}
