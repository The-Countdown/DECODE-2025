package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import java.util.ArrayList;
import java.util.List;

@I2cDeviceType
@DeviceProperties(name = "MCP9808 Temperature Sensor", description = "an MCP9808 temperature sensor", xmlTag = "MCP9808")
public class ADG728 extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    private double[] volteges = new double[9];
    private int activePort;
    private AnalogInput muxAnalog;

    public void writeShort(short value) {
        deviceClient.write8(value);
    }

    // This should not really be needed
    public short readShort() {
        return (short) (deviceClient.read8());
    }

    // Constructor
    public ADG728(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);

        this.deviceClient.setI2cAddress(I2cAddr.create7bit(0x4C));
        this.registerArmingStateCallback(false);

        this.deviceClient.engage();
    }

    // Init
    @Override
    protected synchronized boolean doInitialize() {
        writeShort((short) 0);
        readShort();

        return true;
    }

    public void attachAnalog(AnalogInput muxAnalog) {
        this.muxAnalog = muxAnalog;
    }

    public void setPort(int port) {
        // The open port is set by the high bit from the binary representation of the byte
        activePort = port;
        switch(port) {
            case 1:
                writeShort((short) 254);
            case 2:
                writeShort((short) 253);
            case 3:
                writeShort((short) 251);
            case 4:
                writeShort((short) 247);
            case 5:
                writeShort((short) 239);
            case 6:
                writeShort((short) 223);
            case 7:
                writeShort((short) 191);
            case 8:
                writeShort((short) 127);
        }
        readShort();
    }

    public void setPortRaw(int port) {
        writeShort((short) port);
        readShort();
    }

    public double getTempF(int port) {
        double voltage = volteges[port];
        // Convert to F
        return voltage;
    }

    public double getTempC(int port) {
        double voltage = volteges[port];
        // Convert to C
        return voltage;
    }

    public void saveVoltage() {
        volteges[activePort] = muxAnalog.getVoltage();
    }

    public double[] getVolteges() {
        return volteges;
    }

    // Useless crap
    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Adafruit;
    }

    @Override
    public String getDeviceName() {
        return "Adafruit ADG278 Sensor";
    }
}
