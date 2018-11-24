package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

public class RobotLatch implements RobotMechanic {
    private Servo hook;
    private Servo hangLeg;
    private DcMotor scissorLift;

    private ElapsedTime runtime = new ElapsedTime();

    private volatile boolean scissorLiftBusy = false;
    private volatile boolean scissorLiftUpPosInd = false;

    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getGlobalStore().getHardwareMap();

        hook = hardwareMap.get(Servo.class, "latch");
        hook.setPosition(0.1);

        hangLeg = hardwareMap.get(Servo.class, "hang_leg");
        hangLeg.setPosition(0d);

        scissorLift = hardwareMap.get(DcMotor.class, "lift");
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, scissorLift);
        MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, scissorLift);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, scissorLift);
    }

    public void setHookPosition(double position) {
        hook.setPosition(position);
    }

    public void autoHook() {
        hook.setPosition(1.0);
    }

    public void autoUnhook() {
        hook.setPosition(0.5);
    }

    public void teleHook() {
        hook.setPosition(0.0);
    }

    public void teleUnhook() {
        hook.setPosition(1.0);
    }

    public void adjustHangLegPosition(double position) {
        hangLeg.setPosition(hangLeg.getPosition() + position);
    }

    public void autoLegUp() { hangLeg.setPosition(0.0); }

    public void autoLegDown() {
        hangLeg.setPosition(1.0);
    }

    public void setHang_leg(double position) {
        if(hang_leg.getPosition() < 1 || hang_leg.getPosition() > 0 ) {
            hang_leg.setPosition(hang_leg.getPosition() + position);
        }
    }

    public void powerLift(double power) {
        powerLift(power, -1);
    }

    public void powerLift(double power, int constraint) {
        if(constraint != -1) {
            if(scissorLift.getCurrentPosition() <= constraint) {
                power = 0;
            }
        }

        scissorLift.setPower(power);
    }

    public void powerLiftOnUpPosition(final double power, final int targetPosition) {
        synchronized(scissorLift) {
            if(!scissorLiftBusy) {
                scissorLiftBusy = true;
                if(!scissorLiftUpPosInd) {
                    ThreadUtils.getExecutorService().submit(new Runnable() {
                        @Override
                        public void run() {
                            try {
                                MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, scissorLift);
                                scissorLift.setTargetPosition(targetPosition);
                                scissorLift.setPower(Math.abs(power));

                                runtime.reset();
                                while (OpModeUtils.opModeIsActive() && scissorLift.isBusy()) {
                                    ThreadUtils.idle();
                                }
                            } finally {
                                scissorLiftBusy = false;
                                scissorLiftUpPosInd = true;
                            }
                        }
                    });
                } else {
                    MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    scissorLift.setPower(Math.abs(power));
                }
            }
        }
    }

    public void powerLiftOnDownPosition(final double power, final int targetPosition) {
        synchronized(scissorLift) {
            if(!scissorLiftBusy) {
                scissorLiftBusy = true;
                if(scissorLiftUpPosInd) {
                    ThreadUtils.getExecutorService().submit(new Runnable() {
                        @Override
                        public void run() {
                            try {
                                MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, scissorLift);
                                scissorLift.setTargetPosition(targetPosition);
                                scissorLift.setPower(Math.abs(power));

                                while(OpModeUtils.opModeIsActive() && scissorLift.isBusy()) {
                                    ThreadUtils.idle();
                                }
                            } finally {
                                scissorLiftBusy = false;
                                scissorLiftUpPosInd = false;
                            }
                        }
                    });
                } else {
                    MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    scissorLift.setPower(-1 * Math.abs(power));
                }
            }
        }
    }

    public void unlatch() {
        powerLiftOnUpPosition(0d, 100);
        autoUnhook();
        powerLiftOnDownPosition(0d, 0);
    }

    public void printTelemetry() {
        Telemetry telemetry = OpModeUtils.getGlobalStore().getTelemetry();
        telemetry.addData("lift", scissorLift.getCurrentPosition());
        telemetry.addData("hangLeg", hangLeg.getPosition());
    }
}
