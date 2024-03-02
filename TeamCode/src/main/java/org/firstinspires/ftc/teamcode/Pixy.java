package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.TypeConversion;

@I2cSensor(name = "PixyCam", description = "v1 camera from PixyCam", xmlTag = "Pixy")
public class Pixy extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    @Override
    public Manufacturer getManufacturer(){
        return Manufacturer.valueOf("Charmed Labs");
    }
    @Override
    protected synchronized boolean doInitialize()
    {
        return true;
    }
    @Override
    public String getDeviceName() {
        return "PixyCam v1";
    }
    public Pixy(I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true);

        this.deviceClient.setI2cAddress(I2cAddr.create7bit(0x01));

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();;
    }

    public enum Register
    {
        LARGEST_BLOCK_ALL(0x50),
        SIG_1(0x51),
        SIG_2(0x52),
        SIG_3(0x53),
        SIG_4(0x54),
        SIG_5(0x55),
        SIG_6(0x56),
        SIG_7(0x57);

        public int bVal;

        Register(int bVal)
        {
            this.bVal = bVal;
        }
    }

    protected short readShort(int queryAddress, int bytesToRead)
    {
        return TypeConversion.byteArrayToShort(deviceClient.read(queryAddress, 2));
        /*byte[] first = deviceClient.read(queryAddress, 1);
        byte[] second = deviceClient.read(queryAddress, 1);
        byte[] combined = new byte[first.length + second.length];
        for(int i = 0; i < second.length; i++){
            combined[i] = second[i];
        }
        for(int j = 0; j < first.length; j++){
            combined[j + second.length] = first[j];
        }
        return TypeConversion.byteArrayToShort(combined);*/
    }
}