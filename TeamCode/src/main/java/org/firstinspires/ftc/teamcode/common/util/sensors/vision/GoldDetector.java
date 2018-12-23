package org.firstinspires.ftc.teamcode.common.util.sensors.vision;

import org.firstinspires.ftc.teamcode.autonomous.util.opencv.CameraStream;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfRect;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

public class GoldDetector implements CameraStream.CameraListener, CameraStream.OutputModifier
{

    private Worker worker;
    private Point lastGoldCenter;
    private boolean seen = false;
    private Thread workerThread;

    public GoldDetector()
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
            Mat hsv = new Mat();
            Imgproc.cvtColor(image, hsv, Imgproc.COLOR_BGR2HSV);

            Mat mask = new Mat();
            Core.inRange(hsv, new Scalar(10, 120, 40), new Scalar(33, 255, 255), mask);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat unused = new Mat();
            Imgproc.findContours(mask, contours, unused, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            unused.release();

            overlayData.goldCenters = new ArrayList<>();
            overlayData.contours = new ArrayList<>();
            MatOfPoint bestContour = null;
            double bestArea = 0;
            for (int i = 0; i < contours.size(); i++)
            {
                double area = Imgproc.contourArea(contours.get(i));
                if (area > bestArea)
                {
                    bestArea = area;
                    bestContour = contours.get(i);
                }
                Moments m = Imgproc.moments(contours.get(i));
                Point center = new Point(m.m10 / m.m00,
                        m.m01 / m.m00);
                overlayData.contours.add(contours.get(i));
                overlayData.goldCenters.add(center);
            }
            if (bestContour != null)
            {
                Moments m = Imgproc.moments(bestContour);
                Point center = new Point(m.m10 / m.m00,
                        m.m01 / m.m00);
                overlayData.goldRect = Imgproc.boundingRect(bestContour);
                overlayData.contours.add(bestContour);
                overlayData.goldCenters.add(center);
            }
            mask.release();
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
