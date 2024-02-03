package org.firstinspires.ftc.teamcode.i2cDrivers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

@I2cDeviceType()
public class UltraSonic extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    @Override
    public Manufacturer getManufacturer()
    {

        return Manufacturer.Other;
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        return true;
    }

    @Override
    public String getDeviceName()
    {
        return "MB1242 I2CXL-MaxSonar-EZ4";
    }

    public UltraSonic(I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }



}
