package org.firstinspires.ftc.teamcode.team12538.ext;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class DcMotorWrapper implements DcMotor {
    private String name;
    private DcMotor dcMotor;

    public DcMotorWrapper(String name, HardwareMap hardwareMap) {
        this.name = name;
        this.dcMotor = hardwareMap.get(DcMotor.class, name);
    }

    public String getName() {
        return name;
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return this.dcMotor.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        this.dcMotor.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return this.dcMotor.getController();
    }

    @Override
    public int getPortNumber() {
        return this.dcMotor.getPortNumber();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        this.dcMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return this.dcMotor.getZeroPowerBehavior();
    }

    @Override
    public void setPowerFloat() {
        this.dcMotor.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return this.dcMotor.getPowerFloat();
    }

    @Override
    public void setTargetPosition(int position) {
        this.dcMotor.setTargetPosition(position);
    }

    @Override
    public int getTargetPosition() {
        return this.dcMotor.getTargetPosition();
    }

    @Override
    public boolean isBusy() {
        return this.dcMotor.isBusy();
    }

    @Override
    public int getCurrentPosition() {
        return this.dcMotor.getCurrentPosition();
    }

    @Override
    public void setMode(RunMode mode) {
        this.dcMotor.setMode(mode);
    }

    @Override
    public RunMode getMode() {
        return this.dcMotor.getMode();
    }

    @Override
    public void setDirection(Direction direction) {
        this.dcMotor.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return this.dcMotor.getDirection();
    }

    @Override
    public void setPower(double power) {
        this.dcMotor.setPower(power);
    }

    @Override
    public double getPower() {
        return this.dcMotor.getPower();
    }

    @Override
    public Manufacturer getManufacturer() {
        return this.dcMotor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return this.dcMotor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return this.dcMotor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return this.dcMotor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        this.dcMotor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        this.dcMotor.close();
    }
}
