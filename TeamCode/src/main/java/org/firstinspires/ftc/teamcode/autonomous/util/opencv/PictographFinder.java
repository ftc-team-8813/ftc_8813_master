package org.firstinspires.ftc.teamcode.autonomous.util.opencv;


import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.util.Log;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.autonomous.util.opencv.CameraStream.CameraListener;
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
    private boolean firstRun = true;
    private Mat mat;
    private Mat trainImage;
    private Mat find_mask;
    private Point[] prevQuad = new Point[4];
    private Thread workerThread;
    private MatOfKeyPoint kp_obj;
    private Mat desc_obj;
    private Mat flattened;
    private ClassificationType prevClassification;


    public static class ClassificationType {
        public final String name;
        public final int nLeft, nCenter, nRight;
        private ClassificationType(String name, int nl, int nc, int nr) {
            this.name = name;
            this.nLeft = nl;
            this.nCenter = nc;
            this.nRight = nr;
        }

    }

    public Point[] getPrevQuad() {
        return prevQuad;
    }

    public ClassificationType getPrevClassification() {
        return prevClassification;
    }

    public PictographFinder() {
        //Get an Activity so we can get some image resources
        Activity a = AppUtil.getInstance().getActivity();
        Bitmap bmp = BitmapFactory.decodeResource(a.getResources(), R.drawable.pictograph);
        trainImage = new Mat();
        Utils.bitmapToMat(bmp, trainImage);
        bmp = BitmapFactory.decodeResource(a.getResources(), R.drawable.dot_mask);
        Utils.bitmapToMat(bmp, find_mask);
        bmp.recycle();
        Imgproc.cvtColor(find_mask, find_mask, Imgproc.COLOR_BGR2GRAY);
    }

    @Override
    public void processFrame(Mat rgba) {
        if (firstRun) {
            if (rgba.width() == 0 || rgba.height() == 0) return;
            firstRun = false;
            mat = rgba;
            workerThread = new Thread(new Runnable() {
                @Override
                public void run() {
                    work();
                }
            });
            workerThread.setDaemon(true);
            workerThread.start();
        }
    }

    /**
     * Avoid using this unless it is an emergency. This is not a very clean way to destroy the thread,
     * and *will* cause memory leaks.
     * The processor thread should stop immediately after processing the next frame.
     */
    public void stop() {
        Log.w("PictographFinder", "May need to kill currently running pictograph finder thread. Any errors after this");
        Log.w("PictographFinder", "are completely normal.");
        //We can delete the scene Mat, which should cause enough havoc to kill the thread!
        mat.release();
        //Releasing the Mat does not make it *completely* unusable; just makes it 0 by 0.
        mat = null;
    }

    private void work() {
        //First, we want to find the image
        boolean found = findImage(trainImage);
        if (found) {
            //If we found the image, we need to classify it.
            prevClassification = classify(flattened, find_mask);
        } else {
            //We can't classify it; don't show anything
            prevClassification = new ClassificationType("", -1, -1, -1);
        }
    }

    private ClassificationType classify(Mat img, Mat mask) {
        int mean = (int)Core.mean(img, mask).val[0];
        Imgproc.threshold(img, img, mean, 255, Imgproc.THRESH_BINARY);
        final int[][] dataPoints = {
                {698, 391}, {662, 397}, {315, 416}, {360, 427},
                {700, 426}, {644, 434}, {598, 466}, {450, 509},
                {410, 492}, {379, 479}, {332, 456}, {700, 472},
                {662, 470}, {352, 516}, {314, 500}, {698, 510},
                {655, 510}, {597, 506}
        };
        //                    11111111
        //                    765432109876543210
        //No differences in:        X X
        final int light_c = 0b110011111010101010;
        final int light_l = 0b101101111000000000;
        final int light_r = 0b000110101101011111;
        final int thresh = 80; //Rather dark gray
        int score_l = 0, score_c = 0, score_r = 0;

        for (int i = 0; i < 18; i++) {
            int[] pt = dataPoints[i];
            boolean lc = (light_c & (1 << i)) != 0;
            boolean ll = (light_l & (1 << i)) != 0;
            boolean lr = (light_r & (1 << i)) != 0;

            //Our images are 2x size now for some reason
            int color = (int)(img.get(pt[1]*2, pt[0]*2)[0]);
            boolean l = (color > thresh);
            //May need to weight these?
            score_l += (l == ll) ? 1 : 0;
            score_c += (l == lc) ? 1 : 0;
            score_r += (l == lr) ? 1 : 0;
        }

        boolean lgc = score_l > score_c;
        boolean cgr = score_c > score_r;
        boolean lgr = score_l > score_r;
        ClassificationType choose = new ClassificationType("", score_l, score_c, score_r);
        //  l>c    l>r
        if (lgc && lgr)
            choose = new ClassificationType("Left", score_l, score_c, score_r);
        //        c>l    c>r
        else if (!lgc && cgr)
            choose = new ClassificationType("Center", score_l, score_c, score_r);
        //        r>c     r>l
        else if (!cgr && !lgr)
            choose = new ClassificationType("Right", score_l, score_c, score_r);
        return choose;
    }

    private boolean findImage(Mat find) {
        //We should be able to find ~20-70 matches for good detections, but 10 is enough
        final int MIN_MATCHES = 10;

        Mat object = new Mat();
        find.copyTo(object);
        Mat scene = mat;

        Imgproc.cvtColor(object, object, Imgproc.COLOR_BGR2GRAY);
        Imgproc.cvtColor(scene, scene, Imgproc.COLOR_BGR2GRAY);

        if (mat.width() == 0 || mat.height() == 0) return false;

        //Make a FeatureDetector to find key points
        //We're using the Brisk feature detector/descriptor extractor
        FeatureDetector fd = FeatureDetector.create(FeatureDetector.BRISK);
        //Key points
        Mat temp = new Mat();

        MatOfKeyPoint kp_scene = new MatOfKeyPoint();

        fd.detect(scene, kp_scene, temp); //Find scene key points
        //We only need to find object key points once
        if (kp_obj == null) {
            kp_obj = new MatOfKeyPoint();
            fd.detect(object, kp_obj, temp); //Find object key points
        }
        temp.release();

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
        //Match features on the object to features in the scene
        FlannBasedMatcher matcher = FlannBasedMatcher.create();
        List<MatOfDMatch> matches = new ArrayList<>();
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
        desc_scene.release();
        object.release();
        kp_scene.release();
        return goodMatches.size() > MIN_MATCHES;
    }

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
