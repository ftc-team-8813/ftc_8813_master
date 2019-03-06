package org.firstinspires.ftc.teamcode.common.sensors.vision;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.widget.ImageView;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.List;

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
    public void addListener(CameraListener l)
    {
        super.addListener(l);
    }

    @Override
    public void removeListener(CameraListener l)
    {
        super.removeListener(l);
    }

    @Override
    public void addModifier(OutputModifier m)
    {
        super.addModifier(m);
    }

    @Override
    public void removeModifier(OutputModifier m)
    {
        super.removeModifier(m);
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
        activity.runOnUiThread(new Runnable()
        {
            @Override
            public void run()
            {
                activity.cameraMonitorLayout.addView(view);
            }
        });
        camera.startStreaming(view);
    }

    private void removeFrameView()
    {
        final FtcRobotControllerActivity activity = (FtcRobotControllerActivity) AppUtil.getInstance().getActivity();
        activity.runOnUiThread(new Runnable()
        {
            @Override
            public void run()
            {
                activity.cameraMonitorLayout.removeView(view);
            }
        });
        view.onRemove();
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

            CameraListener[] currentListeners = listeners.toArray(new CameraListener[0]);
            for (CameraListener l : currentListeners)
            {
                Mat copyFrame = new Mat();
                mat.copyTo(copyFrame);
                l.processFrame(copyFrame);
            }
            Mat out = mat;
            OutputModifier[] currentModifiers = modifiers.toArray(new OutputModifier[0]);
            for (OutputModifier m : currentModifiers)
            {
                out = m.draw(out);
            }

            if (!SMURF_MODE)
                Imgproc.cvtColor(out, out, Imgproc.COLOR_BGR2RGBA);
            System.gc();

            final Bitmap outFrame = Bitmap.createBitmap(out.width(), out.height(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(mat, outFrame);
            activity.runOnUiThread(new Runnable()
            {
                @Override
                public void run()
                {
                    setImageBitmap(outFrame);
                }
            });
        }

        void onRemove()
        {
            for (CameraListener l : listeners)
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
