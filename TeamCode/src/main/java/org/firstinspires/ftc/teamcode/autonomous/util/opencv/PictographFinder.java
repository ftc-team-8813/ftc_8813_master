package org.firstinspires.ftc.teamcode.autonomous.util.opencv;


import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.util.Log;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.autonomous.util.telemetry.ProgressBar;
import org.firstinspires.ftc.teamcode.autonomous.util.telemetry.TelemetryWrapper;
import org.firstinspires.ftc.teamcode.autonomous.util.opencv.CameraStream.CameraListener;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.DMatch;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.features2d.DescriptorExtractor;
import org.opencv.features2d.FeatureDetector;
import org.opencv.features2d.FlannBasedMatcher;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * PictographFinder - Detect pictographs in images.
 * Only processes the first image given to it; must be reinitialized afterwards.
 * Note: Uses OpenCV classes, and must be loaded/created AFTER CameraStream is created!
 */

public class PictographFinder implements CameraListener {
    //Is pictograph finding completed?
    private boolean finished = false;
    //Is this the first frame?
    private boolean firstRun = true;
    //The frame
    private Mat mat;
    //The object to look for
    private Mat trainImage;
    //Dot mask
    private Mat find_mask;
    //Box around the image
    private Point[] prevQuad = new Point[4];
    //The worker thread
    private Thread workerThread;
    //The progress bar updater thread
    private Thread progressBarThread;
    //The key points
    private MatOfKeyPoint kp_obj;
    //The descriptors
    private Mat desc_obj;
    //The image taken out of the frame to classify
    private Mat flattened;
    //The classification
    private ClassificationType prevClassification;
    //The progress bar
    private volatile ProgressBar pb;
    //The status message
    private volatile String status;

    private Logger log = new Logger("Pictograph Finder -- Worker");

    /**
     * A pictograph classification result. Contains a name along with the number of left, center,
     * and right matches found.
     */
    public static class ClassificationType {
        /** A display name of the classification (e.g. Left) */
        public final String name;
        /** The number of correlations with the left pictograph dots */
        public final int nLeft;
        /** The number of correlations with the center pictograph dots */
        public final int nCenter;
        /** The number of correlations with the right pictograph dots */
        public final int nRight;

        /* Only this class can create these */
        private ClassificationType(String name, int nl, int nc, int nr) {
            this.name = name;
            this.nLeft = nl;
            this.nCenter = nc;
            this.nRight = nr;
        }

    }

    /**
     * Get the previously detected quad describing the corners of the pictograph in the frame. If
     * no frame was processed or nothing was found, returns an array of zeros.
     * @return The previous quad, or an array of 0 if nothing was found
     */
    public Point[] getPrevQuad() {
        return prevQuad;
    }

    /**
     * Get the previous classification. If nothing was found, returns a classification with "" for
     * the name and all -1's for the scores. If no frame was processed, returns null.
     * @return The previous classification or null if no frame was processed
     */
    public ClassificationType getPrevClassification() {
        return prevClassification;
    }

    /**
     * Whether or not the current classification task is finished.
     * @return true if the worker thread is not busy
     */
    public boolean finished() { return finished; }

    /**
     * Initialize the pictograph finder. Sets the current status to "Init" and retrieve training
     * and mask images.
     */
    public PictographFinder() {
        TelemetryWrapper.setLines(5);
        TelemetryWrapper.setLine(0, "Pictograph Finder -- Init");
        //Get an Activity so we can get some image resources
        Activity a = AppUtil.getInstance().getActivity();
        Bitmap bmp = BitmapFactory.decodeResource(a.getResources(), R.drawable.pictograph);
        trainImage = new Mat();
        find_mask = new Mat();
        Utils.bitmapToMat(bmp, trainImage);
        bmp = BitmapFactory.decodeResource(a.getResources(), R.drawable.dot_mask);
        Utils.bitmapToMat(bmp, find_mask);
        bmp.recycle();
        Imgproc.cvtColor(find_mask, find_mask, Imgproc.COLOR_BGR2GRAY);
    }

    /**
     * Start processing a frame.
     * @param rgba The frame to process
     */
    @Override
    public void processFrame(Mat rgba) {
        if (firstRun) {
            TelemetryWrapper.setLine(0, "Pictograph Finder -- Receive frame");
            if (rgba.width() == 0 || rgba.height() == 0) return;
            firstRun = false;
            //rgba is destroyed after this method ends, so we need to make a copy and use it
            mat = new Mat();
            rgba.copyTo(mat);
            workerThread = new Thread(new Runnable() {
                @Override
                public void run() {
                    work();
                }
            });
            finished = false;
            progressBarThread = new Thread(new Runnable() {

                @Override
                public void run() {
                    while (!finished()) {
                        TelemetryWrapper.setLine(1, "Working - " + pb);
                        TelemetryWrapper.setLine(2, "Status: " + status);
                        try {
                            Thread.sleep(100);
                        } catch (InterruptedException e) {
                            return;
                        }
                    }
                }
            });
            workerThread.setDaemon(true);
            workerThread.start();
            progressBarThread.setDaemon(true);
            progressBarThread.start();
        }
    }

    /**
     * Avoid using this unless it is an emergency. This is not a very clean way to destroy the thread,
     * and *will* cause memory leaks.
     * The processor thread should stop immediately after processing the next frame.
     */
    public void stop() {
        TelemetryWrapper.setLine(0, "Pictograph Finder -- Force Stop");
        finished = true;
        Log.w("PictographFinder", "Force-killing pictograph finder thread. Any errors after this");
        Log.w("PictographFinder", "are completely normal.");
        //We can delete the scene Mat, which should cause enough havoc to kill the thread!
        mat.release();
        //Releasing the Mat does not make it *completely* unusable; just makes it 0 by 0.
        mat = null;
    }

    private void work() {
        // 1: Detect features. 2: Compute descriptors. 3: Find matches. 4: Calculate object position.
        // 5: Classify.
        pb = new ProgressBar(5, 50, '_', '#', '[', ']');
        //First, we want to find the image
        boolean found = findImage(trainImage);
        if (found) {
            //If we found the image, we need to classify it.
            prevClassification = classify(flattened, find_mask);
        } else {
            //We can't classify it; don't show anything
            prevClassification = new ClassificationType("", -1, -1, -1);
        }
        pb.setProgress(5);
        status = "Finished";
        finished = true;
    }

    private ClassificationType classify(Mat img, Mat mask) {
        pb.setProgress(4);
        status = "Classifying object";
        //Take the average color to get a threshold
        int mean = (int)Core.mean(img, mask).val[0];
        //Make the image binary
        Imgproc.threshold(img, img, mean, 255, Imgproc.THRESH_BINARY);
        //Define points to look on the image
        final int[][] dataPoints = {
                {698, 391}, {662, 397}, {315, 416}, {360, 427},
                {700, 426}, {644, 434}, {598, 466}, {450, 509},
                {410, 492}, {379, 479}, {332, 456}, {700, 472},
                {662, 470}, {352, 516}, {314, 500}, {698, 510},
                {655, 510}, {597, 506}
        };

        //Define a bit field for expected light and dark areas. Starts with the last point in
        //dataPoints down to the first.
        //Center
        final int light_c = 0b110011111010101010;
        //Left
        final int light_l = 0b101101111000000000;
        //Right
        final int light_r = 0b000110101101011111;
        //Score for each pictograph type
        int score_l = 0, score_c = 0, score_r = 0;

        for (int i = 0; i < 18; i++) {
            int[] pt = dataPoints[i];
            //Expected values
            boolean lc = (light_c & (1 << i)) != 0;
            boolean ll = (light_l & (1 << i)) != 0;
            boolean lr = (light_r & (1 << i)) != 0;

            //Our images are 2x size now for some reason
            int color = (int)(img.get(pt[1]*2, pt[0]*2)[0]);
            //Actual values
            boolean l = (color > 0);
            //Increase the score if they match
            score_l += (l == ll) ? 1 : 0;
            score_c += (l == lc) ? 1 : 0;
            score_r += (l == lr) ? 1 : 0;
        }
        //short names
        int l = score_l;
        int c = score_c;
        int r = score_r;
        //A blank one if it is not the same
        ClassificationType choose = new ClassificationType("", score_l, score_c, score_r);
        //Choose the maximum of them
        if (l > c && l > r)
            choose = new ClassificationType("Left", score_l, score_c, score_r);
        else if (c > l && c > r)
            choose = new ClassificationType("Center", score_l, score_c, score_r);
        else if (r > l && r > c)
            choose = new ClassificationType("Right", score_l, score_c, score_r);
        return choose;
    }

    private boolean findImage(Mat find) {
        long start = System.currentTimeMillis();
        //We should be able to find ~20-70 matches for good detections, but 10 is enough
        final int MIN_MATCHES = 10;

        Mat object = new Mat();
        find.copyTo(object);
        Mat scene = mat;

        Imgproc.cvtColor(object, object, Imgproc.COLOR_BGR2GRAY);
        Imgproc.cvtColor(scene, scene, Imgproc.COLOR_BGR2GRAY);

        if (mat.width() == 0 || mat.height() == 0) return false;

        status = "Detecting Features";

        //Make a FeatureDetector to find key points
        //We're using the Brisk feature detector/descriptor extractor algorithms
        FeatureDetector fd = FeatureDetector.create(FeatureDetector.BRISK);
        //Key points
        Mat temp = new Mat();

        MatOfKeyPoint kp_scene = new MatOfKeyPoint();

        //Detect key points
        fd.detect(scene, kp_scene, temp); //Find scene key points
        //We only need to find object key points once
        if (kp_obj == null) {
            kp_obj = new MatOfKeyPoint();
            fd.detect(object, kp_obj, temp); //Find object key points
        }
        temp.release();

        pb.setProgress(1);
        status = "Computing Descriptors";

        //Descriptors
        Mat desc_scene = new Mat();

        //Make a DescriptorExtractor to find descriptors
        DescriptorExtractor de = DescriptorExtractor.create(DescriptorExtractor.BRISK);
        if (desc_obj == null) {
            desc_obj = new Mat();
            de.compute(object, kp_obj, desc_obj);
            desc_obj.convertTo(desc_obj, CvType.CV_32F);
        }
        de.compute(scene, kp_scene, desc_scene); //Find scene descriptors

        //Convert descriptors to CV_32F
        desc_scene.convertTo(desc_scene, CvType.CV_32F);

        pb.setProgress(2);
        status = "Finding matches";

        //Match features on the object to features in the scene
        FlannBasedMatcher matcher = FlannBasedMatcher.create();
        List<MatOfDMatch> matches = new ArrayList<>();
        //Use K nearest neighbor matching
        matcher.knnMatch(desc_obj, desc_scene, matches, 2);

        //Sort out matches that make sense
        List<DMatch> goodMatches = new ArrayList<>();
        KeyPoint[] ko = kp_obj.toArray(), ks = kp_scene.toArray();
        for (int i = 0; i < matches.size(); i++) {
            DMatch[] ms = matches.get(i).toArray();
            if (matches.size() < 2)
                continue;
            //Good matches should be close together?
            if (ms[0].distance <= 0.7 * ms[1].distance)
                goodMatches.add(ms[0]);
        }
        //If we have at least MIN_MATCHES matches, we have found the object!
        if (goodMatches.size() >= MIN_MATCHES) {

            pb.setProgress(3);
            status = "Calculating object position";

            List<Point> objPts = new ArrayList<>();
            List<Point> scenePts = new ArrayList<>();

            for (int i = 0; i < goodMatches.size(); i++) {
                objPts.add(ko[goodMatches.get(i).queryIdx].pt);
                scenePts.add(ks[goodMatches.get(i).trainIdx].pt);
            }

            Mat h = Calib3d.findHomography(new MatOfPoint2f(objPts.toArray(new Point[0])),
                    new MatOfPoint2f(scenePts.toArray(new Point[0])), Calib3d.RANSAC, 3);

            Point[] pts = getCorners(object, h);

            Mat inverse = h.inv();
            Mat inv = new Mat();
            Imgproc.warpPerspective(scene, inv, inverse, object.size(), Imgproc.INTER_LINEAR, Core.BORDER_CONSTANT, new Scalar(0));
            prevQuad = pts;
            if (flattened != null) flattened.release();
            flattened = inv;
        } else {
            prevQuad = new Point[4];
        }
        //Release objects to save our tiny bit of memory
        desc_scene.release();
        object.release();
        kp_scene.release();
        long end = System.currentTimeMillis();
        log.i("Finding completed in %.3f seconds", (end-start)/1000.0);
        return goodMatches.size() >= MIN_MATCHES;
    }

    //Get the corners of an object
    private Point[] getCorners(Mat object, Mat h) {
        List<Point> objCorners = new ArrayList<>();
        objCorners.add(new Point(0, 0));
        objCorners.add(new Point(object.cols(), 0));
        objCorners.add(new Point(object.cols(), object.rows()));
        objCorners.add(new Point(0, object.rows()));
        MatOfPoint2f sc = new MatOfPoint2f();
        Core.perspectiveTransform(new MatOfPoint2f(objCorners.toArray(new Point[0])),
                sc, h);
        Point[] pts = sc.toArray();
        sc.release();
        return pts;
    }
}
