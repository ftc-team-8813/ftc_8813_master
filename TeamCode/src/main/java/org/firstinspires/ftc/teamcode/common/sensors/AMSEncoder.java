package org.firstinspires.ftc.teamcode.common.sensors;

import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.firstinspires.ftc.teamcode.common.util.concurrent.GlobalThreadPool;

import java.util.concurrent.Future;

/**
 * I2CDevice wrapper for the AMS AS5048B magnetic encoder.
 * Based on the Arduino library
 */
@I2cDeviceType
@DeviceProperties(xmlTag="AMSEncoder", name="AMS AS5048B Encoder",
        compatibleControlSystems=ControlSystem.REV_HUB)
public class AMSEncoder extends I2cDeviceSynchDevice<I2cDeviceSynch>
{
    public static final int I2C_ADDRESS = 0x40;

    private boolean error = false;
    private I2cDeviceSynch device;
    private Logger log;
    private double prevAngle = 0;
    private long prevSampleTime = 0;
    private int rotations = 0;

    private int zeroPos = 0;
    
    @Override
    protected boolean doInitialize()
    {
        error = false;
        int test = read8Timeout(0xFE, 500);
        if (test < 0)
        {
            log.w("Device not connected!");
            error = true;
            return false;
        }
    
        device.setReadWindow(
                new I2cDeviceSynch.ReadWindow(0xFE, 2, I2cDeviceSynch.ReadMode.REPEAT)
        );
    
        device.write8(0x16, 0);
        device.write8(0x17, 0);
        return true;
    }
    
    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.AMS;
    }
    
    @Override
    public String getDeviceName()
    {
        return "AS5048B";
    }
    
    private class Timeout
    {
        private Thread toInterrupt;
        private Thread timeoutThread;
        private long millis;
        private Logger log;

        public Timeout(long millis)
        {
            this(millis, Thread.currentThread());
        }

        public Timeout(long millis, Thread toInterrupt)
        {
            this.toInterrupt = toInterrupt;
            this.millis = millis;
            this.log = new Logger("Timeout (" + toInterrupt + ")");
        }

        public void start()
        {
            timeoutThread = new Thread(() ->
            {
                try
                {
                    Thread.sleep(millis);
                    toInterrupt.interrupt();
                    log.v("Timed out after " + millis + " ms");
                }
                catch (InterruptedException e)
                {
                    log.v("Interrupted");
                }
            });
            timeoutThread.setDaemon(true);
            timeoutThread.start();
        }

        public void stop()
        {
            if (timeoutThread != null) timeoutThread.interrupt();
        }
    }

    private int read8Timeout(int rgadr, int timeout)
    {
        Timeout t = new Timeout(timeout);
        int val;
        t.start();
        val = device.read8(rgadr);
        if (Thread.interrupted()) // Clear the interrupt flag
        {
            return -1;
        }
        t.stop();
        return val;
    }

    public AMSEncoder(I2cDeviceSynch device)
    {
        super(device, true);
        this.device = device;
        device.setI2cAddress(I2cAddr.create7bit(I2C_ADDRESS));
        super.registerArmingStateCallback(false);
        device.engage();
        log = new Logger("AMSEncoder");
    }

    public void resetEncoder()
    {
        if (error) return;

        zeroPos = getRawAngle();
        rotations = 0;
    }

    public int getRawAngle()
    {
        if (error) return -1;
        // byte[] data = device.read(0xFE, 2);
        int msb = device.read8(0xFF);
        int lsb = device.read8(0xFE);
        // log.v("Should be 2 bytes: " + data.length);
        return ((msb & 0xFF) << 6) + (lsb & 0x3F);
    }

    public double getAngle()
    {
        if (error) return -1;
        double angle;
        long elapsed = System.nanoTime()/1000000L - prevSampleTime;
        
        if (elapsed > 5)
        {
            angle = (double)((getRawAngle() - zeroPos) * 360) / 0x3FFF;
        }
        else
        {
            angle = prevAngle;
        }

        if (elapsed < 200)
        {
            if (Math.abs(angle - prevAngle) > 180
            )
            {
                rotations += Math.signum(prevAngle - angle);
            }
        }
        else
        {
            log.w("Too much time between samples (" + elapsed + "); some rotations may have been missed");
        }
        prevAngle = angle;
        prevSampleTime = System.nanoTime()/1000000L;

        return angle;
    }

    public double getAbsoluteAngle()
    {
        if (error) return -1;
        return getAngle() + 360 * rotations;
    }

    public int getRotations()
    {
        return rotations;
    }

    public boolean error()
    {
        return error;
    }
}
