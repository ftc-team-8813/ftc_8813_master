package org.firstinspires.ftc.teamcode.common.util;

import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.videoio.VideoWriter;

import java.io.File;

public class Vlogger
{
    private VideoWriter writer;
    private volatile boolean closed;
    private String filename;


    public Vlogger(String filename, int width, int height, double fps)
    {
        this.filename = filename;
        int fourcc = VideoWriter.fourcc('M', 'J', 'P', 'G');
        writer = new VideoWriter(Config.storageDir + filename, fourcc, fps, new Size(width, height));
    }

    public synchronized void put(Mat frame)
    {
        if (closed) return;
        writer.write(frame);
    }

    public synchronized void close()
    {
        closed = true;
        writer.release();
        Utils.scanFile(new File(Config.storageDir + filename));
    }
}
