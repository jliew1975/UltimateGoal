package org.firstinspires.ftc.teamcode.team12538.TurnTableApp;

import org.firstinspires.ftc.teamcode.team12538.TurnTableApp.TurnTable.Direction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Emily Liew on 8/22/18.
 */

public class TurnTableApp extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        TurnTable turnTable = new TurnTable(0.8);

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.a) {
                turnTable.stop();
                turnTable.start(0.3, Direction.ClockWise);
            } else if(gamepad1.b) {
                turnTable.stop();
                turnTable.start(0.3, Direction.CounterClockWise);
            }

            if(gamepad1.dpad_up) {
                turnTable.increasePower();
            } else if(gamepad1.dpad_down) {
                turnTable.decreasePower();
            }

            if(gamepad1.x) {
                turnTable.stop();
            }

            turnTable.print(telemetry);
        }
    }
}
