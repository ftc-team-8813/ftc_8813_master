package org.firstinspires.ftc.teamcode.autonomous.util.opencv;

import android.app.Activity;
import android.view.View;
import android.widget.LinearLayout;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcontroller.internal.EventHooks;
import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * CameraStream - Listen for camera input and return Mat frames when it is available. This class
 * **MUST** be initialized (e.g. by CameraStream.class) BEFORE using ANY OpenCV classes!!!
 */

public class CameraStream implements EventHooks {

    private JavaCameraView cameraView;

    private List<CameraListener> listeners = new ArrayList<>();

    //Default modifier: do nothing to the image
    public static final OutputModifier defaultModifier =
            new OutputModifier() { public Mat process(Mat rgba) {return rgba;} };
    private OutputModifier modifier = defaultModifier;

    public void addListener(CameraListener l) {
        listeners.add(l);
    }

    public void removeListener(CameraListener l) {
        listeners.remove(l);
    }

    public void setModifier(OutputModifier m) {
        modifier = m;
    }

    @Override
    public void stop() {
        cameraView.disableView();
    }

    @Override
    public void resume() {
        cameraView.enableView();
    }

    public static interface CameraListener {
        public void processFrame(Mat rgba);
    }

    public static interface OutputModifier {
        public Mat process(Mat rgba);
    }

    private void startProcessing() {
        cameraView.setCvCameraViewListener(new CameraBridgeViewBase.CvCameraViewListener2() {
            @Override
            public void onCameraViewStarted(int width, int height) { }

            @Override
            public void onCameraViewStopped() { }

            @Override
            public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
                Mat frame = inputFrame.rgba();
                //Convert to BGR to make it a 'normal' image
                Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGBA2BGR);
                //Make a copy of the current listeners list to try to be more thread-safe
                CameraListener[] currentListeners = listeners.toArray(new CameraListener[0]);
                for (CameraListener l : currentListeners) {
                    //Make a copy of the frame so that processors cannot modify the image
                    Mat copyFrame = new Mat();
                    frame.copyTo(copyFrame);
                    l.processFrame(frame);
                    copyFrame.release();
                }
                Mat out =  modifier.process(frame);
                //Run the garbage collector as fast as possible to delete old images and keep enough
                //memory for our program to function, avoid blowing up the phone :)
                System.gc();
                return out;
            }
        });
    }

    static {
        if (!OpenCVLoader.initDebug()) {
            System.exit(0);
        }
    }

    public CameraStream() {
        Activity activity = AppUtil.getInstance().getActivity();
        FtcRobotControllerActivity rc = (FtcRobotControllerActivity) activity;
        LinearLayout cameraLayout = rc.cameraMonitorLayout;
        cameraView = new JavaCameraView(activity, JavaCameraView.CAMERA_ID_BACK);
        cameraView.setVisibility(View.INVISIBLE);
        cameraLayout.addView(cameraView);
        BaseAutonomous.instance().addEventHooks(this);
    }
}
