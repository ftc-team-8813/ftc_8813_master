package org.firstinspires.ftc.teamcode.autonomous.tasks;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.util.opencv.CameraStream;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.sensors.IMU;
import org.firstinspires.ftc.teamcode.common.sensors.vision.ShapeGoldDetector;
import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.firstinspires.ftc.teamcode.common.util.Profiler;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class TaskSample implements Task, CameraStream.OutputModifier
{
    private ShapeGoldDetector detector;
    private final DcMotor left, right;
    private Telemetry telemetry;
    private boolean continuous = false;
    private static final boolean onWebcam = true;
    private Logger log = new Logger("TaskSample");

    public TaskSample()
    {
        Robot robot = Robot.instance();
        left = robot.leftFront;
        right = robot.rightFront;
        telemetry = BaseAutonomous.instance().telemetry;
    }

    public TaskSample(ShapeGoldDetector detector)
    {
        this();
        this.detector = detector;
    }

    public TaskSample(ShapeGoldDetector detector, boolean continuous)
    {
        this(detector);
        this.continuous = continuous;
    }

    @Override
    public void runTask() throws InterruptedException
    {
        CameraStream stream = BaseAutonomous.instance().getCameraStream();
        if (detector == null)
        {
            detector = new ShapeGoldDetector();
            stream.addModifier(detector);
            stream.addListener(detector);
        }
        stream.addModifier(this);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Thread.sleep(1000); // Wait for the camera to warm up

        long lostTime = System.currentTimeMillis();
        double lpower = 0;
        boolean seen = false;
        Robot robot = Robot.instance();

        log.v("Attempting to sample");
        while (continuous || seen || System.currentTimeMillis() - lostTime < 2000)
        {
            if (robot.imu.getStatus() == IMU.STARTED) robot.imu.update();
            telemetry.update();
            telemetry.clearAll();
            telemetry.addData("Gold on-screen: ", detector.goldSeen());
            if (detector.getLocation() != null)
            {
                telemetry.addData("Location", detector.getLocation());

                // If our phone is mounted upside down, horizontal = +Y
                // If we're on a webcam, horizontal = +X
                if (!detector.goldSeen())
                {
                    if (seen)
                    {
                        lostTime = System.currentTimeMillis();
                        log.v("Mineral lost");
                        seen = false;
                    }
                    lpower -= 0.01;
                    left.setPower(lpower);
                    right.setPower(lpower);
                    continue;
                }
                if (!seen) log.v("Mineral found!");
                seen = true;
                lpower = 0.1;
                // Horizontal error, normalized
                double e;
                if (onWebcam)
                {
                    e = -(detector.getLocation().x / 640 - 0.5) * 2;
                }
                else
                {
                    e = -(detector.getLocation().y / 480 - 0.5) * 2;
                }
                telemetry.addData("Error", e);

                // Bias
                double l =  0.15;
                double r =  0.15;

                // Turn amount
                if (e < 0)
                {
                    r += e * 0.2;
                    l -= e * 0.2;
                }
                else
                {
                    l -= e * 0.2;
                    r += e * 0.2;
                }

                telemetry.addData("Left", l);
                telemetry.addData("Right", r);
                telemetry.addData("Error", e);
                log.v("Error: %.4f; Left: %.4f, Right: %.4f", e, l, r);

                left.setPower(l*2);
                right.setPower(r*2);
            }
            Thread.sleep(1); // Throws InterruptedException properly
        }
        stream.removeModifier(detector);
        stream.removeListener(detector);
        stream.removeModifier(this);
    }

    @Override
    public Mat draw(Mat bgr)
    {
        Point location = detector.getLocation();
        if (location != null)
        {
            // Vertical line (blue)
            Imgproc.arrowedLine(bgr, new Point(location.x, 0), new Point(location.x, 479), new Scalar(0, 0, 255), 1, 8, 0, 0.01);
            // Horizontal line (red)
            Imgproc.arrowedLine(bgr, new Point(0, location.y), new Point(639, location.y), new Scalar(255, 0, 0), 1, 8, 0, 0.01);
        }
        return bgr;
    }
}
