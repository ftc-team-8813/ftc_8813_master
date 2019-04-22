package org.firstinspires.ftc.teamcode.common.util;

import java.io.BufferedOutputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.charset.Charset;

public class DataLogger implements AutoCloseable
{

    private File file;
    private DataOutputStream logger;
    private int numChannels;
    private long clipStart;
    private volatile Thread logThread;
    private volatile boolean error = false;

    private Logger log = new Logger("DataLogger");

    public static class Channel
    {
        public final String name;
        public final int color;

        public Channel(String name, int color)
        {
            this.name = name;
            this.color = color;
        }
    }

    public static interface LogCallback
    {
        public void putData(double[] array);
    }

    public DataLogger(File f, Channel... channels)
    {
        try
        {
            this.file = f;
            logger = new DataOutputStream(new BufferedOutputStream(new FileOutputStream(f)));
            numChannels = channels.length;
            writeHeader(channels);
        }
        catch (IOException e)
        {
            log.w("Error opening datalogger stream");
            log.w(e);
            error = true;
        }
    }

    private void writeHeader(Channel[] channels) throws IOException
    {
        logger.write("LOGp".getBytes(Charset.forName("UTF-8")));
        logger.writeInt(numChannels);
        for (int i = 0; i < numChannels; i++)
        {
            logger.writeInt(channels[i].color);
            logger.write(channels[i].name.getBytes(Charset.forName("UTF-8")));
            logger.write(0x00); // Null termination
        }
    }

    public synchronized void startClip()
    {
        if (error) return;
        try
        {
            logger.writeDouble(Double.NaN);
            clipStart = System.nanoTime();
        }
        catch (IOException e)
        {
            log.w("Unable to write timestamp");
            log.w(e);
            error = true;
        }
    }

    public synchronized void log(double[] data)
    {
        if (error) return;
        if (clipStart == 0) return;
        try
        {
            logger.writeLong(System.nanoTime() - clipStart);
            for (double d : data)
            {
                logger.writeDouble(d);
            }
        }
        catch (IOException e)
        {
            log.w("Unable to write data");
            log.w(e);
            error = true;
        }
    }

    public synchronized void startLogging(final LogCallback callback)
    {
        if (logThread != null) throw new IllegalStateException("Logging thread already running!");
        if (error) return;
        logThread = new Thread(() ->
        {
            while (!Thread.interrupted())
            {
                double[] channels = new double[numChannels];
                callback.putData(channels);
                log(channels);
                Thread.yield();
            }
            logThread = null;
        });
        logThread.setDaemon(true);
        logThread.start();
    }

    public synchronized void stopLogging()
    {
        if (logThread != null) logThread.interrupt();
    }

    @Override
    public void close()
    {
        if (logThread != null)
        {
            logThread.interrupt();
            try
            {
                logThread.join();
            }
            catch (InterruptedException e) { }
        }
        try
        {
            logger.close();
        }
        catch (IOException e)
        {
            log.w("Unable to close logger");
            log.w(e);
        }
        Utils.scanFile(file);
    }

    public boolean error()
    {
        return error;
    }
}
