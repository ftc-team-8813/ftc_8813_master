package org.firstinspires.ftc.teamcode.autonomous.util.opencv;

import org.firstinspires.ftc.teamcode.common.sensors.vision.CameraStream;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;

import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class SkystoneDetector implements CameraStream.CameraListener, CameraStream.OutputModifier
{
    static
    {
        System.loadLibrary("vision");
    }
    
    private Future<DetectResult> currentWorker;
    
    private ExecutorService worker;
    
    private DetectResult currentResult;
    
    public SkystoneDetector()
    {
        worker = Executors.newSingleThreadExecutor();
    }
    
    @Override
    public void processFrame(Mat bgr)
    {
        if (currentWorker.isDone())
        {
            try
            {
                currentResult = currentWorker.get();
            } catch (ExecutionException | InterruptedException e)
            {
                e.printStackTrace();
            }
        }
        if (currentWorker == null || currentWorker.isDone())
        {
            currentWorker = worker.submit(new Worker(bgr));
        }
        bgr.release();
    }
    
    @Override
    public void stop()
    {
        if (currentWorker != null)
        {
            currentWorker.cancel(true);
        }
    }
    
    @Override
    public Mat draw(Mat bgr)
    {
        return null;
    }
    
    
    private class DetectResult
    {
        public boolean detected;
        public Rect area;
    }
    
    private class Worker implements Callable<DetectResult>
    {
        private Mat bgr;
        
        public Worker(Mat bgr)
        {
            this.bgr = bgr.clone();
        }
    
        @Override
        public DetectResult call()
        {
            int status = submit(bgr.getNativeObjAddr());
            bgr.release();
            if (status < 0)
            {
                return null;
            }
            
            DetectResult result = new DetectResult();
            result.detected = detected();
            if (!result.detected)
            {
                return result;
            }
            
            result.area = new Rect(new Point(get_min_x(), get_min_y()), new Point(get_max_x(), get_max_y()));
            
            return result;
        }
    }
    
    private native int submit(long mat_addr);
    private native boolean detected();
    private native int get_min_x();
    private native int get_max_x();
    private native int get_min_y();
    private native int get_max_y();
}
