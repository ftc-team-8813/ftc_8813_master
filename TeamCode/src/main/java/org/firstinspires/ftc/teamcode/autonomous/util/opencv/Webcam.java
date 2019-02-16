package org.firstinspires.ftc.teamcode.autonomous.util.opencv;

import android.graphics.Bitmap;
import android.support.annotation.NonNull;

import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.common.util.Logger;

import java.util.List;
import java.util.concurrent.Executor;
import java.util.concurrent.TimeUnit;

public class Webcam
{

    public enum Status
    {
        INIT, OPENING, OPEN, STREAMING, CLOSED, ERROR
    }

    private CameraManager manager;
    private CameraName name;

    private int format;
    private Size size;
    private Camera camera;
    private volatile CameraCharacteristics characteristics;

    private Executor executor;

    private volatile Status status = Status.INIT;

    private Logger log = new Logger("Webcam");

    public Webcam(CameraName name)
    {
        this.manager = Webcam.getCameraManager();
        this.name = name;
        executor = ThreadPool.newSingleThreadExecutor("Webcam Lifecycle Handler");
    }

    public boolean open(int deadline)
    {
        log.d("Attempting to open camera (deadline=%d)", deadline);
        Deadline dl = new Deadline(deadline, TimeUnit.SECONDS);
        status = Status.OPENING;
        camera = manager.requestPermissionAndOpenCamera(dl, name, Continuation.create(executor, new StatusCallback()));
        if (camera == null)
        {
            log.d("Open failed");
            status = Status.ERROR;
            return false;
        }
        return true;
    }

    public int[] getSupportedFormats()
    {
        return characteristics.getAndroidFormats();
    }

    public Size[] getSupportedSizes(int format)
    {
        return characteristics.getSizes(format);
    }

    public Size getDefaultSize(int format)
    {
        return characteristics.getDefaultSize(format);
    }

    public void setFormat(int format, Size size) throws IllegalArgumentException
    {
        if (!Misc.contains(getSupportedFormats(), format))
            throw new IllegalArgumentException("Unsupported format: " + format);
        if (size == null)
            throw new IllegalArgumentException("size must not be null!");
        boolean hasSize = false;
        for (Size s : getSupportedSizes(format))
        {
            if (s.getWidth() == size.getWidth() && s.getHeight() == size.getHeight())
            {
                hasSize = true;
                break;
            }
        }
        if (!hasSize) throw new IllegalArgumentException("Unsupported size: " + size);

        this.format = format;
        this.size = size;
    }

    public void startStreaming(FrameCallback callback)
    {
        if (size == null) throw new IllegalStateException("Please set the format first!");
        status = Status.STREAMING;
        try
        {
            camera.createCaptureSession(Continuation.create(executor, new CaptureStatusCallback(callback)));
        } catch (CameraException e)
        {
            log.e("Error creating capture session");
            log.e(e);
            camera.close();
            status = Status.ERROR;
        }
    }

    public void close()
    {
        if (camera != null)
        {
            camera.close();
            status = Status.CLOSED;
        }
    }

    public static List<WebcamName> getConnectedWebcams()
    {
        return getCameraManager().getAllWebcams();
    }

    private static CameraManager getCameraManager()
    {
        return ClassFactory.getInstance().getCameraManager();
    }

    private class CaptureStatusCallback extends CameraCaptureSession.StateCallbackDefault
    {
        private FrameCallback frameCallback;

        public CaptureStatusCallback(FrameCallback frameCallback)
        {
            this.frameCallback = frameCallback;
        }

        @Override
        public void onConfigured(@NonNull CameraCaptureSession session)
        {
            try
            {
                CameraCaptureRequest request = camera.createCaptureRequest(format, size, characteristics.getMaxFramesPerSecond(format, size));

                CameraCaptureSequenceId sequenceId = session.startCapture(request,
                        new CaptureCallback(request, frameCallback),
                        Continuation.create(executor, new CameraCaptureSession.StatusCallback()
                        {
                            @Override
                            public void onCaptureSequenceCompleted(@NonNull CameraCaptureSession session, CameraCaptureSequenceId cameraCaptureSequenceId, long lastFrameNumber)
                            {
                                log.d("Capture sequence completed");
                            }
                        }));
            } catch (CameraException e)
            {
                log.e("Error creating capture request");
                log.e(e);
                camera.close();
                status = Status.ERROR;
            }
        }

        @Override
        public void onClosed(@NonNull CameraCaptureSession session)
        {
            log.d("Camera session closed");
        }
    }

    public static interface FrameCallback
    {
        public void onFrame(Bitmap frame);
    }

    private class CaptureCallback implements CameraCaptureSession.CaptureCallback
    {
        private Bitmap bitmap;
        private FrameCallback callback;

        public CaptureCallback(CameraCaptureRequest request, FrameCallback callback)
        {
            bitmap = request.createEmptyBitmap();
            this.callback = callback;
        }

        @Override
        public void onNewFrame(@NonNull CameraCaptureSession session, @NonNull CameraCaptureRequest request, @NonNull CameraFrame cameraFrame)
        {
            cameraFrame.copyToBitmap(bitmap);
            callback.onFrame(bitmap);
        }
    }

    private class StatusCallback implements Camera.StateCallback
    {

        @Override
        public void onOpened(@NonNull Camera camera)
        {
            status = Status.OPEN;
            log.d("Camera opened");
            characteristics = name.getCameraCharacteristics();
            log.d("Supported configurations:");
            for (String line : characteristics.toString().split("\\n"))
            {
                log.d(line);
            }
        }

        @Override
        public void onOpenFailed(@NonNull CameraName cameraName, @NonNull Camera.OpenFailure reason)
        {
            log.e("Open failed -- reason: %s", reason.toString());
        }

        @Override
        public void onClosed(@NonNull Camera camera)
        {
            log.d("Camera closed");
            if (status != Status.ERROR) status = Status.CLOSED;
        }

        @Override
        public void onError(@NonNull Camera camera, Camera.Error error)
        {
            log.e("Camera error: ", error.toString());
            if (error.equals(Camera.Error.Disconnected))
            {
                status = Status.CLOSED;
            } else
            {
                status = Status.ERROR;
            }
            camera.close();
        }
    }
}
