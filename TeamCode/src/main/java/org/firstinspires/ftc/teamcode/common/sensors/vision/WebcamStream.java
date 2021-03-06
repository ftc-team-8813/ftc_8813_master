package org.firstinspires.ftc.teamcode.common.sensors.vision;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.widget.ImageView;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.common.util.GlobalDataLogger;
import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class WebcamStream extends CameraStream
{

    private Logger log = new Logger("Webcam Stream");
    private FrameView view;
    private Webcam camera;

    public WebcamStream()
    {
        super();
    }

    @Override
    public Size getSize()
    {
        return new Size(640, 480);
    }

    @Override
    protected void init()
    {
        Logger log = new Logger("WebcamTest");
        List<WebcamName> webcams = Webcam.getConnectedWebcams();
        if (webcams.size() == 0) throw new IllegalStateException("No webcams available");
        else
        {
            log.d("Available cameras:");
            for (WebcamName name : webcams)
            {
                log.d(name.getUsbDeviceNameIfAttached());
            }
        }
        camera = new Webcam(webcams.get(0));
        boolean opened = camera.open(60);
        for (int i = 0; i < 5 && !opened; i++)
        {
            log.w("Failed to open camera; trying again (attempt=%d)", i);
            opened = camera.open(10);
        }
        if (!opened) throw new IllegalStateException("Unable to open camera; please restart the robot");
        try
        {
            Thread.sleep(5);
        } catch (InterruptedException e)
        {
            log.w("InterruptedException");
        }
        camera.setFormat(ImageFormat.YUY2, camera.getDefaultSize(ImageFormat.YUY2));
        addFrameView();
    }

    @Override
    public void stop()
    {
        log.d("Stopping");
        removeFrameView();
        camera.close();
    }

    private void addFrameView()
    {
        final FtcRobotControllerActivity activity = (FtcRobotControllerActivity) AppUtil.getInstance().getActivity();
        view = new WebcamStream.FrameView(activity);
        activity.runOnUiThread(() -> activity.cameraMonitorLayout.addView(view));
        camera.startStreaming(view);
        // GlobalDataLogger.instance().addChannel("Camera Exposure", () -> "" + camera.getCamera().getControl(ExposureControl.class).getExposure(TimeUnit.MILLISECONDS));
    }

    private void removeFrameView()
    {
        final FtcRobotControllerActivity activity = (FtcRobotControllerActivity) AppUtil.getInstance().getActivity();
        activity.runOnUiThread(() -> activity.cameraMonitorLayout.removeView(view));
        view.onRemove();
    }
    
    public Camera getInternalCamera()
    {
        return camera.getCamera();
    }

    private class FrameView extends ImageView implements Webcam.FrameCallback
    {
        private Activity activity;
        private boolean closed = false;

        public FrameView(Activity context)
        {
            super(context);
            activity = context;
        }

        @Override
        public void onFrame(final Bitmap frame)
        {
            Mat mat = new Mat();
            Utils.bitmapToMat(frame, mat);
            Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGBA2BGR);

            Mat out = processFrame(mat);
            
            final Bitmap outFrame = Bitmap.createBitmap(out.width(), out.height(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(mat, outFrame);
            activity.runOnUiThread(() -> setImageBitmap(outFrame));
        }

        void onRemove()
        {
            for (CameraListenerWrapper l : listeners)
            {
                l.stop();
            }
        }

        @Override
        public void onClose(Webcam.Status status)
        {
            if (!closed)
            {
                log.w("Camera closed!");
                closed = true;
                stop();
            }
        }
    }
}
