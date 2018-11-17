package org.firstinspires.ftc.teamcode.util.sensors.vision;

import org.firstinspires.ftc.teamcode.autonomous.util.opencv.CameraStream;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfRect;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

public class ShapeGoldDetector implements CameraStream.CameraListener, CameraStream.OutputModifier
{

    private Worker worker;
    private Point lastGoldCenter;
    private boolean seen = false;
    private Thread workerThread;

    public ShapeGoldDetector()
    {
        worker = new Worker();
        workerThread = new Thread(worker, "Gold Detector Worker");
        workerThread.start();
    }

    @Override
    public void processFrame(Mat bgr)
    {
        worker.setFrame(bgr);
    }

    public Point getLocation()
    {
        return lastGoldCenter;
    }

    public boolean goldSeen()
    {
        return seen;
    }

    @Override
    public Mat draw(Mat bgr)
    {
        OverlayData data = worker.overlayData.clone();
        if (data.contours != null && data.goldRect != null)
        {
            Imgproc.drawContours(bgr, data.contours, -1, new Scalar(0, 255, 0), 2);
            if (data.contours.size() > 0) Imgproc.drawContours(bgr, data.contours, data.contours.size()-1, new Scalar(255, 0, 0), 2);
            Rect r = data.goldRect;
            Imgproc.rectangle(bgr, new Point(r.x, r.y), new Point(r.x + r.width, r.y + r.height), new Scalar(255, 255, 0), 2);
        }
        if (data.goldCenters != null)
        {
            for (int i = 0; i < data.goldCenters.size(); i++)
            {
                Imgproc.circle(bgr, data.goldCenters.get(i), 5, new Scalar(0, 0, 255));
            }
            if (data.goldCenters.size() > 0)
            {
                lastGoldCenter = data.goldCenters.get(data.goldCenters.size()-1);
                seen = true;
            }
            else
            {
                seen = false;
            }
        }
        return bgr;
    }

    @Override
    public void stop()
    {
        if (workerThread.isAlive()) workerThread.interrupt();
    }

    private class Worker implements Runnable
    {

        private volatile Mat mat;
        public volatile boolean running;
        public volatile OverlayData overlayData = new OverlayData();

        @Override
        public void run()
        {
            while (true)
            {
                try
                {
                    work();
                    Thread.sleep(5);
                }
                catch (InterruptedException e)
                {
                    return;
                }
            }
        }

        public void setFrame(Mat frame)
        {
            if (!running) mat = frame;
            else frame.release();
        }

        private void work()
        {
            if (running || mat == null) return;
            running = true;
            process(mat.clone());
            running = false;
            mat.release();
            mat = null;
        }

        private void process(Mat image)
        {
            Mat gray = new Mat();
            Core.extractChannel(image, gray, 2); // Extract the red channel
            Imgproc.blur(gray, gray, new Size(5, 5));

            // Detect edges
            Mat edges = new Mat();
            Imgproc.Canny(gray, edges, 75, 195);
            //Imgproc.approx

            image.release();
            System.gc();
        }
    }

    private class OverlayData implements Cloneable
    {
        public volatile List<MatOfPoint> contours;
        public volatile List<Point> goldCenters;
        public volatile Rect goldRect;
        public volatile List<Point> silverCenters;
        public volatile MatOfRect silverDetected;

        public OverlayData clone()
        {
            OverlayData out = new OverlayData();
            if (contours != null)       out.contours = new ArrayList<>(contours);
            if (goldCenters != null)    out.goldCenters = new ArrayList<>(goldCenters);
            if (goldRect != null)       out.goldRect = goldRect.clone();
            if (silverCenters != null)  out.silverCenters = new ArrayList<>(silverCenters);
            if (silverDetected != null)
            {
                out.silverDetected = new MatOfRect();
                silverDetected.copyTo(out.silverDetected);
            }
            return out;
        }
    }
}

