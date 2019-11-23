package org.firstinspires.ftc.teamcode.common.sensors.vision;

import android.app.Activity;
import android.view.View;
import android.widget.LinearLayout;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Vector;

/**
 * CameraStream - Listen for camera input and return Mat frames when it is available. This class
 * **MUST** be initialized (e.g. by CameraStream.class) BEFORE using ANY OpenCV classes!!!
 */

public class CameraStream
{
    static
    {
        if (!OpenCVLoader.initDebug())
        {
            System.exit(0);
        }
    }
    
    private JavaCameraView cameraView;
    private Activity activity;
    private volatile boolean uiRunning;
    //Smurf mode -- swap red and blue color channels in the output image for EPIC results :)
    protected static final boolean SMURF_MODE = false;
    
    protected volatile List<CameraListenerWrapper> listeners = new Vector<>();

    public Size getSize()
    {
        return new Size(480, 640);
    }
    
    public void addListener(CameraListener l)
    {
        listeners.add(new CameraListenerWrapper(l, 0));
        listeners.sort(Comparator.naturalOrder());
    }
    
    public void removeListener(CameraListener l)
    {
        for (int i = 0; i < listeners.size(); i++)
        {
            if (listeners.get(i).listener == l)
            {
                listeners.remove(i);
                break;
            }
        }
        l.stop();
    }
    
    public void addModifier(OutputModifier m)
    {
        listeners.add(new CameraListenerWrapper(m, 1000));
        listeners.sort(Comparator.naturalOrder());
    }

    public void removeModifier(OutputModifier m)
    {
        for (int i = 0; i < listeners.size(); i++)
        {
            if (listeners.get(i).modifier == m)
            {
                listeners.remove(i);
                break;
            }
        }
    }
    
    public void stop()
    {
        for (CameraListenerWrapper l : listeners)
        {
            l.stop();
        }
        listeners.clear();

        FtcRobotControllerActivity rc = (FtcRobotControllerActivity) activity;
        final LinearLayout cameraLayout = rc.cameraMonitorLayout;
        uiRunning = true;
        activity.runOnUiThread(() ->
        {
            cameraView.disableView();
            cameraLayout.removeView(cameraView);
            uiRunning = false;
        });
    }
    
    protected class CameraListenerWrapper implements Comparable<CameraListenerWrapper>
    {
        final CameraListener listener;
        final OutputModifier modifier;
        int priority;
        
        CameraListenerWrapper(CameraListener listener, int priority)
        {
            this.listener = listener;
            this.modifier = null;
            this.priority = priority;
        }
    
        CameraListenerWrapper(OutputModifier modifier, int priority)
        {
            this.modifier = modifier;
            this.listener = null;
            this.priority = priority;
        }
        
        void stop()
        {
            if (listener != null) listener.stop();
        }
        
        Mat process(Mat bgr)
        {
            if (listener != null)
            {
                Mat clone = bgr.clone();
                listener.processFrame(clone);
                clone.release();
                return bgr;
            }
            else if (modifier != null)
            {
                return modifier.draw(bgr);
            }
            return null;
        }
    
        @Override
        public int compareTo(CameraListenerWrapper other)
        {
            return Integer.compare(priority, other.priority);
        }
    }
    
    public static interface CameraListener
    {
        public void processFrame(Mat bgr);
        public void stop();
    }
    
    public static interface OutputModifier
    {
        public Mat draw(Mat bgr);
    }
    
    private void startProcessing()
    {
        cameraView.setVisibility(View.VISIBLE);
        cameraView.setCameraPermissionGranted();
        cameraView.setCvCameraViewListener(new CameraBridgeViewBase.CvCameraViewListener2()
        {
            @Override
            public void onCameraViewStarted(int width, int height)
            {
            }
            
            @Override
            public void onCameraViewStopped()
            {
            }
            
            @Override
            public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame)
            {
                Mat frame = inputFrame.rgba();
                if (frame.width() == 0 || frame.height() == 0) return frame;
                //Log.i("Image Conversion", "Image type: " + CvType.typeToString(frame.type()));
                //Convert to BGR to make it a 'normal' image
                Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGBA2BGR);
                
                return processFrame(frame);
            }
        });
    }
    
    protected Mat processFrame(Mat frame)
    {
        //Make a copy of the current listeners list to try to be more thread-safe
        CameraListenerWrapper[] currentListeners = listeners.toArray(new CameraListenerWrapper[0]);
        for (CameraListenerWrapper l : currentListeners)
        {
            frame = l.process(frame);
        }
    
        if (!SMURF_MODE)
            Imgproc.cvtColor(frame, frame, Imgproc.COLOR_BGR2RGBA);
        //Run the garbage collector as fast as possible to delete old images and keep enough
        //memory for our program to function, avoid blowing up the phone :)
        System.gc();
        return frame;
    }

    protected void init()
    {
        Logger log = new Logger("Camera Stream Initializer");
        log.v("Adding camera view to screen");
        long begin = System.currentTimeMillis();
        activity = AppUtil.getInstance().getActivity();
        FtcRobotControllerActivity rc = (FtcRobotControllerActivity) activity;
        final LinearLayout cameraLayout = rc.cameraMonitorLayout;
        uiRunning = true;
        activity.runOnUiThread(() ->
        {
            cameraView = new JavaCameraView(activity, JavaCameraView.CAMERA_ID_BACK);
            cameraView.setVisibility(View.INVISIBLE);
            //cameraView.setRotation(90);
            cameraLayout.addView(cameraView);
            uiRunning = false;
        });
        while (uiRunning)
        {
            try
            {
                Thread.sleep(0, 100);
            } catch (InterruptedException e) {
                log.w("Ignoring interrupt");
            }
        }
        startProcessing();
        cameraView.enableView();
    }
    
    public CameraStream()
    {
        init();
    }
}
