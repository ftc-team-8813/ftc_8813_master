package org.firstinspires.ftc.teamcode.autonomous.test.opencv;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.widget.ImageView;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.common.sensors.vision.Webcam;
import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.common.util.Logger;

import java.io.File;
import java.io.IOException;
import java.util.List;

@Autonomous(name="Webcam Test")
@Disabled
public class WebcamTest extends LinearOpMode
{
    private Webcam camera;
    private FrameView view;

    @Override
    public void runOpMode() throws InterruptedException
    {
        try
        {
            Logger.init(new File(Config.storageDir + "webcam_info.txt"));
        } catch (IOException e)
        {
            e.printStackTrace();
        }
        Logger log = new Logger("WebcamTest");
        List<WebcamName> webcams = Webcam.getConnectedWebcams();
        if (webcams.size() == 0)
        {
            telemetry.addData("No webcams available", "");
            telemetry.update();
            return;
        }
        else
        {
            log.d("Available cameras:");
            for (WebcamName name : webcams)
            {
                log.d(name.getUsbDeviceNameIfAttached());
            }
        }
        camera = new Webcam(webcams.get(0));
        camera.open(60);
        Thread.sleep(5);
        camera.setFormat(ImageFormat.YUY2, camera.getDefaultSize(ImageFormat.YUY2));
        waitForStart();
        addFrameView();
        while (opModeIsActive())
        {
            Thread.sleep(5);
        }
        camera.close();
        Logger.close();
    }

    private void addFrameView()
    {
        final FtcRobotControllerActivity activity = (FtcRobotControllerActivity) AppUtil.getInstance().getActivity();
        view = new FrameView(activity);
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
    }

    private class FrameView extends ImageView implements Webcam.FrameCallback
    {
        private Activity activity;

        public FrameView(Activity context)
        {
            super(context);
            activity = context;
        }

        @Override
        public void onFrame(final Bitmap frame)
        {
            activity.runOnUiThread(new Runnable()
            {
                @Override
                public void run()
                {
                    setImageBitmap(frame);
                }
            });
        }

        @Override
        public void onClose(Webcam.Status status)
        {
            removeFrameView();
        }
    }
}
