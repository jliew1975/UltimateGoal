package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

public class RobotOuttake implements RobotComponent, ControlAware, TelemetryAware {

    public RobotStoneClaw outtakeClaw = new RobotStoneClaw();
    public RobotOuttakeSlides outtakeSlides = new RobotOuttakeSlides();

    private volatile boolean busy = false;
    private volatile boolean inReadyState = false;

    @Override
    public void init() {
        outtakeClaw.init();
        outtakeSlides.init();
    }

    @Override
    public void control(Gamepad gamepad) {

    }

    @Override
    public void printTelemetry() {

    }

    private synchronized void readyForFoundationDeployment() {
        if(!busy) {
            busy = true;

            if(!inReadyState) {
                ThreadUtils.getExecutorService().submit(new Runnable() {
                    @Override
                    public void run() {
                        try {
                            // clam the stone with the claw
                            outtakeClaw.setClawPosition(RobotStoneClaw.CLAW_CLAM_POSITION);

                            // raise outtake slide
                            outtakeSlides.runToPosition(300);

                            // swing outtake arm out for stone deployment
                            outtakeClaw.setArmPosition(RobotStoneClaw.ARM_DEPLOYMENT_POSITION);
                            inReadyState = true;
                        } finally {
                            busy = false;
                        }
                    }
                });
            }
        }
    }
}
