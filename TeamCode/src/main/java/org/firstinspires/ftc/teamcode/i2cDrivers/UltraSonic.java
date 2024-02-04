package org.firstinspires.ftc.teamcode.i2cDrivers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

@I2cDeviceType
@DeviceProperties(name = "MaxSonar-EZ4", xmlTag = "MB1242")
public class UltraSonic extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x70);

    public enum Register {
        WRITE(0xE0),
        REPORT_LAST_VAL(0xE1),
        RANGE_READING(0x51),
        CHANGE_SENSOR_ADDRESS1(0xAA),
        CHANGE_SENSOR_ADDRESS2(0xA5);

        public int bVal;

        Register(int bVal) {
            this.bVal = bVal;
        }
    }



    @Override
    public Manufacturer getManufacturer() {
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

    public UltraSonic(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned)
    {
        super(deviceClient, true);

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    protected void writeShort(final Register reg, short value) {
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    protected short readShort(Register reg) {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }

    public void rangeReading() {
        int rangeRead = Register.RANGE_READING.bVal;
        writeShort(Register.WRITE, (short) rangeRead);
    }

    public boolean isReading() {
        return true;
    }

    public double reportRangeReadingCM() {
        return readShort(Register.REPORT_LAST_VAL);
    }

    public double reportRangeReadingIN() {
        return readShort(Register.REPORT_LAST_VAL)/2.54;
    }






}
