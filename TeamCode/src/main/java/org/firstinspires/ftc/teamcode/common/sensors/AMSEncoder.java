package org.firstinspires.ftc.teamcode.common.sensors;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.teamcode.common.util.Logger;

/**
 * I2CDevice wrapper for the AMS AS5048B magnetic encoder.
 * Based on the Arduino library
 */
public class AMSEncoder
{
    public static final int I2C_ADDRESS = 0x40;

    private boolean error = false;
    private I2cDeviceSynch device;
    private Logger log;
    private double prevAngle = 0;
    private long prevSampleTime = 0;
    private int rotations = 0;

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
                    return;
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
        this.device = device;
        device.setI2cAddress(I2cAddr.create7bit(I2C_ADDRESS));
        device.engage();
        log = new Logger("AMSEncoder");

        int test = read8Timeout(0xFE, 500);
        if (test < 0)
        {
            log.w("Device not connected!");
            error = true;
            return;
        }

        device.setReadWindow(
                new I2cDeviceSynch.ReadWindow(0xFE, 2, I2cDeviceSynch.ReadMode.REPEAT)
        );
        prevAngle = getAngle();
    }

    public void resetEncoder()
    {
        if (error) return;
        device.write8(0x16, 0);
        device.write8(0x17, 0);

        int angle = getRawAngle();
        device.write8(0x16, (angle >> 6) & 0xFF);
        device.write8(0x17, angle & 0x3F);

        prevAngle = 0;
        rotations = 0;
    }

    public int getRawAngle()
    {
        if (error) return -1;
        byte[] data = device.read(0xFE, 2);
        return (((int)data[0] & 0xFF) << 8) + ((int)data[1] & 0xFF);
    }

    public double getAngle()
    {
        if (error) return -1;
        double angle = (double)(getRawAngle() * 360) / 0xFFFF;
        long elapsed = System.currentTimeMillis() - prevSampleTime;

        if (elapsed < 50)
        {
            if (Math.abs(angle - prevAngle) > 270)
            {
                rotations += Math.signum(angle - prevAngle);
            }
        }
        else
        {
            log.w("Too much time between samples (" + elapsed + "); some rotations may have been missed");
        }
        prevAngle = angle;
        prevSampleTime = System.currentTimeMillis();

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
}
