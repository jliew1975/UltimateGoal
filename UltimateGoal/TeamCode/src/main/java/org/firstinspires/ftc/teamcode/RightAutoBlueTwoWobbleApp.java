package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue (R) - 2 Wobble", group="Group 1")
public class RightAutoBlueTwoWobbleApp extends RightAutoBlueApp {
    public RightAutoBlueTwoWobbleApp() {
        super.isPickupSecondWobbleGoal = true;
    }
}
