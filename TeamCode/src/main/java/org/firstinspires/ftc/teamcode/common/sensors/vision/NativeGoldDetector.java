package org.firstinspires.ftc.teamcode.common.sensors.vision;

import org.firstinspires.ftc.teamcode.autonomous.util.opencv.CameraStream;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

public class NativeGoldDetector implements CameraStream.CameraListener, CameraStream.OutputModifier
{
    static
    {
        System.loadLibrary("native-lib");
    }

    private Worker worker;
    private Thread workerThread;

    public NativeGoldDetector()
    {
        worker = new Worker();
        workerThread = new Thread(new Worker());
    }

    public native void draw(long mat_addr);

    public native void process(long mat_addr);

    public native int getX();

    public native int getY();

    public native boolean seen();

    private class Worker implements Runnable
    {

        public volatile Mat inputFrame;
        public volatile Mat outputFrame;
        public final Object lock = new Object();

        @Override
        public void run()
        {
            while (true)
            {
                if (inputFrame != null)
                {
                    Mat copy = new Mat();
                    inputFrame.copyTo(copy);
                    process(copy.getNativeObjAddr());
                    copy.release();
                    inputFrame.release();
                    inputFrame = null;
                    if (outputFrame != null) outputFrame.setTo(new Scalar(0, 0, 0));
                    else outputFrame = new Mat(640, 480, CvType.CV_8UC3);
                    draw(outputFrame);
                }
                try
                {
                    Thread.sleep(1);
                } catch (InterruptedException e)
                {
                    break;
                }

            }
        }
    }

    @Override
    public void processFrame(Mat bgr)
    {

    }

    @Override
    public void stop()
    {

    }

    @Override
    public Mat draw(Mat bgr)
    {
        if (worker.outputFrame != null && !worker.outputFrame.empty())
            Core.add(bgr, worker.outputFrame, bgr);
        return bgr;
    }
}
