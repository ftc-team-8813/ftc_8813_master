package org.firstinspires.ftc.teamcode.common.util;

import java.io.DataOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.charset.Charset;

public class DataLogger implements AutoCloseable
{

    private DataOutputStream logger;
    private int numChannels;
    private long clipStart;
    private volatile Thread logThread;

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

    public DataLogger(File f, Channel... channels) throws IOException
    {
        logger = new DataOutputStream(new FileOutputStream(f));
        numChannels = channels.length;
        writeHeader(channels);
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

    public synchronized void startClip() throws IOException
    {
        logger.writeDouble(Double.NaN);
        clipStart = System.nanoTime();
    }

    public synchronized void log(double[] data) throws IOException
    {
        if (clipStart == 0) return;
        logger.writeLong(System.nanoTime() - clipStart);
        for (double d : data)
        {
            logger.writeDouble(d);
        }
    }

    public synchronized void startLogging(final LogCallback callback)
    {
        if (logThread != null) throw new IllegalStateException("Logging thread already running!");
        logThread = new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                while (!Thread.interrupted())
                {
                    double[] channels = new double[numChannels];
                    callback.putData(channels);
                    try
                    {
                        log(channels);
                    } catch (IOException e)
                    {
                        break;
                    }
                    Thread.yield();
                }
                logThread = null;
            }
        });
        logThread.setDaemon(true);
        logThread.start();
    }

    public synchronized void stopLogging()
    {
        logThread.interrupt();
    }

    @Override
    public void close() throws IOException
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
        logger.close();
    }
}
