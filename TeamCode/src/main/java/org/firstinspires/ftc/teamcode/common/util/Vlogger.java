package org.firstinspires.ftc.teamcode.common.util;

import org.firstinspires.ftc.teamcode.common.sensors.vision.CameraStream;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.videoio.VideoWriter;

import java.io.File;

public class Vlogger implements CameraStream.CameraListener
{
    private VideoWriter writer;
    private boolean closed;
    private String filename;
    private Logger log = new Logger("Vlogger");
    private boolean empty = true;


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
        empty = false;
    }

    public synchronized void close()
    {
        closed = true;
        writer.release();
        log.d("Video saved to %s", filename);
        if (empty)
        {
            new File(Config.storageDir + filename).delete();
            log.d("Video has no content; deleted");
        }
        else
        {
            Utils.scanFile(new File(Config.storageDir + filename));
        }
    }
    
    @Override
    public void processFrame(Mat bgr)
    {
        put(bgr);
    }
    
    @Override
    public void stop()
    {
        close();
    }
}
