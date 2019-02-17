package org.firstinspires.ftc.teamcode.common.util.sensors.vision;

import org.firstinspires.ftc.teamcode.autonomous.util.opencv.CameraStream;
import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfRect;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Rect2d;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.opencv.tracking.Tracker;
import org.opencv.tracking.TrackerCSRT;
import org.opencv.tracking.TrackerGOTURN;
import org.opencv.tracking.TrackerKCF;
import org.opencv.tracking.TrackerMOSSE;
import org.opencv.tracking.TrackerMedianFlow;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Shape-based mineral detector and tracker.
 * <h3>1: How it works</h3>
 * The detector uses several criteria to detect gold minerals:
 * <ol>
 *     <li>Color: The detector first creates a binary image from the input frame, selecting areas that
 *         are yellow. It then uses findContours() to find the edges of all of these areas.</li>
 *     <li>Perimeter: If the area is really small, it is most probably noise.</li>
 *     <li>Edge count: Uses contour approximation to approximate the polygonal shape of the area. If
 *         this results in a shape that is between 4 (cube straight on) and 6 (cube at an angle) edges,
 *         it is probably a cube; otherwise, it probably isn't. This has a tendency to fail if the cube
 *         is too far away, has too much pure-white glare, or is obstructed in some other way.</li>
 *     <li>Convexity: If the polygon is not convex, it probably isn't a cube (but it might just be
 *         obstructed).</li>
 *     <li>Aspect ratio: If the area is too wide or tall, it definitely isn't a cube.</li>
 *     <li>Upper limit: If there is not</li>
 * </ol>
 * After an object has been detected, the program switches to using OpenCV's {@link TrackerMOSSE MOSSE tracker}.
 * That can track the mineral much more quickly and efficiently than repeating the detection process. Once the tracker
 * cannot find the mineral any more (or the area being tracked is not yellow; the tracker can get confused), the
 * program switches back to detection.
 * <br>
 * <br>
 * The detection runs in a separate thread so as to not bog down the camera thread and drop frames.
 * This causes some visual discrepancy between the detection output lines and the frame they are
 * overlaid on, but otherwise it should not cause any issues.
 * <h3>2: Usage</h3>
 * The ShapeGoldDetector is very easy to use. Simply add it as a listener and modifier to the CameraStream:
 * <pre><code>
 *     stream.addListener(detector);
 *     stream.addModifier(detector);
 * </code></pre>
 * Then it will automatically start looking for minerals in the camera stream. The location of the block
 * and whether it can be seen can be easily accessed through the {@link #getLocation()} and {@link #goldSeen()}
 * functions. The detector will automatically be stopped when the camera stream closes.
 *
 */
public class ShapeGoldDetector implements CameraStream.CameraListener, CameraStream.OutputModifier
{

    private final boolean onWebcam;

    private Worker worker;
    private Point lastGoldCenter;
    private boolean seen = false;
    private Thread workerThread;

    private static final double ASPECT_TOLERANCE = 0.4;
    private static final int MIN_Y = 100;

    /**
     * Construct a ShapeGoldDetector. Assumes that it is running on a webcam by default.
     */
    public ShapeGoldDetector()
    {
        this(true);
    }

    /**
     * Construct a ShapeGoldDetector.
     * @param onWebcam Set to false if the camera stream is coming from the phone
     */
    public ShapeGoldDetector(boolean onWebcam)
    {
        this.onWebcam = onWebcam;
        worker = new Worker();
        workerThread = new Thread(worker, "ShapeGoldDetector Worker");
        workerThread.start();
    }

    @Override
    public void processFrame(Mat bgr)
    {
        worker.setFrame(bgr);
    }

    /**
     * Get the location where the mineral was last seen.
     * @return The location of the mineral. Can be null if the mineral has not been seen yet.
     */
    public Point getLocation()
    {
        return lastGoldCenter;
    }

    /**
     * Get the current detection status.
     * @return true if the mineral is detected and being tracked; false otherwise.
     */
    public boolean goldSeen()
    {
        return seen;
    }

    @Override
    public Mat draw(Mat bgr)
    {
        OverlayData data = worker.overlayData.clone();
        if (data.contours != null)
        {
            int i = 0;
            for (MatOfPoint contour : new ArrayList<>(data.contours))
            {
                if (contour == null) continue;
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                double perim = Imgproc.arcLength(contour2f, true);
                int edgeCount = contour.height();
                boolean convex = Imgproc.isContourConvex(contour);
                Rect rect = Imgproc.boundingRect(contour);
                double aspectRatio = (double)rect.height / rect.width;

                Scalar color = new Scalar(0, 127, 0);
                int thickness = 2;

                // COLOR CODING:
                // Thin -- Too small
                // Red -- Wrong number of edges (green otherwise)
                // Yellow -- Bad aspect ratio
                // Dark -- Concave
                // Teal -- Selected

                if (perim < 100)
                {
                    thickness = 1;
                }
                if (edgeCount < 4 || edgeCount > 6)
                {
                    color = new Scalar(0, 0, 127);
                }
                if (aspectRatio < 1 - ASPECT_TOLERANCE || aspectRatio > 1 + ASPECT_TOLERANCE)
                {
                    color = new Scalar(0, 127, 127);
                }
                if (i == data.contours.size()-1)
                {
                    color = new Scalar(127, color.val[1], color.val[2]);
                }
                if (convex)
                {
                    color = new Scalar(color.val[0]*2, color.val[1]*2, color.val[2]*2);
                }
                Imgproc.polylines(bgr, Arrays.asList(contour), true, color, thickness);

                contour2f.release();
                i++;
            }
        }
        if (data.goldRect != null)
        {
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
        if (onWebcam)
        {
            Imgproc.line(bgr, new Point(0, MIN_Y), new Point(640, MIN_Y), new Scalar(255, 255, 255), 2);
        }
        else
        {
            Imgproc.line(bgr, new Point(640 - MIN_Y, 0), new Point(640 - MIN_Y, 480), new Scalar(255, 255, 255), 2);
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

        private Logger log = new Logger("ShapeGoldDetector Worker");
        private volatile Mat mat;
        public volatile boolean running;
        public volatile OverlayData overlayData = new OverlayData();

        private Tracker tracker;
        private Rect2d boundingBox;
        private int fails;

        @Override
        public void run()
        {
            tracker = TrackerMOSSE.create();
            int iter = 0;
            while (true)
            {
                try
                {
                    work();
                    Thread.sleep(5);
                    if (iter > 50) System.gc();
                    iter++;
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
            if (boundingBox != null) track(getTrackingImage(mat), mat.clone());
            else process(mat.clone());
            running = false;
            mat.release();
            mat = null;
        }

        private void track(Mat image, Mat color)
        {
            overlayData.contours.clear();
            overlayData.goldCenters = new ArrayList<>();
            boolean ret = tracker.update(image, boundingBox);
            // Tracker can get stuck on any edge
            if (!ret || boundingBox.y + boundingBox.height > 580 || boundingBox.x < -100 || boundingBox.y < -100
                    || boundingBox.x + boundingBox.width > 740 || !validateTrackingRect(color, boundingBox))
            {
                if (fails > 3)
                {
                    // Reload the tracker to reset it, otherwise it will refuse to detect
                    tracker.clear();
                    tracker = TrackerMOSSE.create();

                    boundingBox = null;
                    overlayData.goldRect = null;
                    fails = 0;
                }
                else
                {
                    fails++;
                    if (boundingBox.width != 0 && boundingBox.height != 0)
                    {
                        overlayData.goldRect = new Rect((int) boundingBox.x, (int) boundingBox.y,
                                (int) boundingBox.width, (int) boundingBox.height);
                        overlayData.goldCenters.add(
                                new Point(boundingBox.x + boundingBox.width / 2, boundingBox.y + boundingBox.height / 2));
                    }
                }
            }
            else
            {
                fails = 0;
                overlayData.goldRect = new Rect((int)boundingBox.x, (int)boundingBox.y,
                                                (int)boundingBox.width, (int)boundingBox.height);
                overlayData.goldCenters.add(
                        new Point(boundingBox.x + boundingBox.width/2, boundingBox.y + boundingBox.height / 2));
            }
            image.release();
            color.release();
        }

        private boolean validateTrackingRect(Mat input, Rect2d r)
        {
            Mat roi = input.submat(new Rect((int)Math.max(r.x, 0), (int)Math.max(r.y, 0),
                    (int)Math.min(r.width, 639 - r.x), (int)Math.min(r.height, 479 - r.y)));

            Scalar avg = Core.mean(roi);
            // Convert color to HSV
            Mat mean = new Mat(1, 1, CvType.CV_8UC3);
            mean.put(0, 0, avg.val[0], avg.val[1], avg.val[2]);
            Imgproc.cvtColor(mean, mean, Imgproc.COLOR_BGR2HSV);

            avg = new Scalar(mean.get(0, 0));
            mean.release();

            // Check if the colors are within range
            boolean inRange = avg.val[0] >= 10 && avg.val[0] <= 33
                            && avg.val[1] >= 70 && avg.val[1] <= 255
                            && avg.val[2] >= 40 && avg.val[2] <= 255;

            if (inRange)
            {
                System.out.format("validateTrackingRect succeeded with HSV <%.0f, %.0f, %.0f>\n", avg.val[0], avg.val[1], avg.val[2]);
            }
            else
            {
                System.out.format("validateTrackingRect failed with HSV <%.0f, %.0f, %.0f>\n", avg.val[0], avg.val[1], avg.val[2]);
            }


            return inRange;
        }

        private Mat getTrackingImage(Mat input)
        {
            Mat dst = new Mat();
            Core.extractChannel(input, dst, 0); // Extract the blue channel for max contrast
            return dst;
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
                MatOfPoint2f contour2f = new MatOfPoint2f(contours.get(i).toArray());
                double perim = Imgproc.arcLength(contour2f, true);
                MatOfPoint2f poly = new MatOfPoint2f();
                Imgproc.approxPolyDP(contour2f, poly, 0.015 * perim, true);

                MatOfPoint ipoly = new MatOfPoint(poly.toArray());
                overlayData.contours.add(ipoly);
                Rect rect = Imgproc.boundingRect(ipoly);
                double aspectRatio = (double)rect.height / rect.width;

                Moments m = Imgproc.moments(ipoly);
                Point center = new Point(m.m10 / m.m00,
                        m.m01 / m.m00);

                if (perim >= 100 && poly.height() >= 4 && poly.height() <= 6
                        && Imgproc.isContourConvex(ipoly)
                        && aspectRatio >= 1 - ASPECT_TOLERANCE && aspectRatio <= 1 + ASPECT_TOLERANCE
                        && ((onWebcam && center.y > MIN_Y) || (!onWebcam && center.x < 640 -  MIN_Y)))
                {
                    double area = Imgproc.contourArea(poly);
                    if (bestArea < area)
                    {
                        bestContour = ipoly;
                        bestArea = area;
                    }
                }
                contours.get(i).release();
                poly.release();
                contour2f.release();
            }
            if (bestContour != null)
            {
                Moments m = Imgproc.moments(bestContour);
                Point center = new Point(m.m10 / m.m00,
                        m.m01 / m.m00);

                overlayData.goldRect = Imgproc.boundingRect(bestContour);
                boundingBox = new Rect2d(overlayData.goldRect.x, overlayData.goldRect.y,
                                        overlayData.goldRect.width, overlayData.goldRect.height);
                tracker.init(getTrackingImage(image), boundingBox);
                overlayData.contours.add(bestContour);
                overlayData.goldCenters.add(center);
            }
            else
            {
                overlayData.goldRect = null;
                overlayData.goldCenters.clear();
            }
            mask.release();
            image.release();
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
