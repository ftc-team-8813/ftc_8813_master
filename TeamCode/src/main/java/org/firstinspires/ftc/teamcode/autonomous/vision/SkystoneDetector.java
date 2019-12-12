package org.firstinspires.ftc.teamcode.autonomous.vision;

import org.firstinspires.ftc.teamcode.common.sensors.vision.CameraStream;
import org.firstinspires.ftc.teamcode.common.util.GlobalDataLogger;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;

import java.util.concurrent.Callable;
import java.util.concurrent.CancellationException;
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
    
        GlobalDataLogger.instance().addChannel("Skystone detected", () -> found() ? "1" : "0");
        
    }
    
    @Override
    public void processFrame(Mat bgr)
    {
        if (currentWorker != null && currentWorker.isDone())
        {
            try
            {
                currentResult = currentWorker.get();
            } catch (ExecutionException | CancellationException | InterruptedException e)
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
        draw(bgr.nativeObj);
        return bgr;
    }
    
    public boolean found()
    {
        if (currentResult == null) return false;
        return currentResult.detected;
    }
    
    public Rect getArea()
    {
        if (!found()) return null;
        return currentResult.area;
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
            int status = submit(bgr.nativeObj);
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
    
    private native void draw(long mat_addr);
}

