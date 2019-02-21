package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskDetectGold;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskDrop;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskIntakeMineral;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskSample;
import org.firstinspires.ftc.teamcode.autonomous.util.opencv.CameraStream;
import org.firstinspires.ftc.teamcode.autonomous.util.opencv.WebcamStream;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.firstinspires.ftc.teamcode.common.util.Profiler;
import org.firstinspires.ftc.teamcode.common.util.Utils;
import org.firstinspires.ftc.teamcode.common.util.Vlogger;
import org.firstinspires.ftc.teamcode.common.sensors.vision.ShapeGoldDetector;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.io.File;

@Autonomous(name="Crater Autonomous")
public class MainAutonomous extends BaseAutonomous implements CameraStream.OutputModifier
{

    private Vlogger video;
    private Logger log;
    private volatile String state;
    private volatile long start;

    private static final int LEFT = -1;
    private static final int CENTER = 0;
    private static final int RIGHT = 1;

    private int side;

    private Profiler profiler = new Profiler();

    public static final boolean DROP = false;

    @Override
    public void initialize() throws InterruptedException
    {
        Robot robot = Robot.instance();
        robot.hook.setPosition(Robot.HOOK_CLOSED);
        robot.imu.initialize(telemetry);
        robot.imu.start();
        CameraStream stream = getCameraStream();
        video = new Vlogger(getVlogName(),
                (int)stream.getSize().width, (int)stream.getSize().height, 10.0);
        log = new Logger("Crater Autonomous");
        robot.initPivotAuto();
    }

    private String getVlogName()
    {
        int i;
        for (i = 0; new File(Config.storageDir + "autonomous_capture" + i + ".avi").exists(); i++);
        return "autonomous_capture" + i + ".avi";
    }

    @Override
    public void run() throws InterruptedException
    {
        profiler.start("run");
        start = System.currentTimeMillis();
        state = "Initializing";

        Robot robot = Robot.instance();
        robot.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize camera
        profiler.start("init camera");
        CameraStream stream = getCameraStream();
        ShapeGoldDetector detector = new ShapeGoldDetector();
        profiler.end();

        profiler.start("drop");
        if (DROP) new TaskDrop().runTask();
        else Thread.sleep(4000);
        profiler.end();

        DcMotor left = robot.leftFront;
        DcMotor right = robot.rightFront;
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Start detecting after the camera has warmed up
        stream.addModifier(detector);
        stream.addListener(detector);
        stream.addModifier(this);

        robot.imu.resetHeading();

        state = "Detecting Mineral";
        profiler.start("detect");
        new TaskDetectGold(detector, profiler).runTask();
        profiler.end();

        state = "Sampling Mineral";
        profiler.start("sample");
        new TaskSample(detector).runTask();
        profiler.end();
        telemetry.clearAll();
        telemetry.update();

        robot.imu.update();
        if (robot.imu.getHeading() >= 25) side = LEFT;
        else if (robot.imu.getHeading() <= -30) side = RIGHT;
        else side = CENTER;

        state = "Intake mineral";
        profiler.start("intake");
        new TaskIntakeMineral(profiler).runTask();
        profiler.end();

        state = "Park in crater";
        profiler.start("park");
        profiler.start("forward");
        robot.forward(24, 0.5);
        Thread.sleep(100);
        profiler.end();

        // Drop the intake
        profiler.start("drop");
        robot.pivot.stopHolding();
        Thread.sleep(15);
        log.d("Starting intake drop");
        robot.intakePivot.setPower(0.5);
        Thread.sleep(400);
        robot.intakePivot.setPower(0);
        profiler.end();
        profiler.end(); // park
        profiler.end(); // run

    }

    @Override
    public synchronized void finish()
    {
        video.close();
        log.d("Crater autonomous finished -- mineral=%s, drop=%s", side, Boolean.toString(DROP));
        profiler.finish();
    }

    @Override
    public synchronized Mat draw(Mat bgr)
    {
        Mat frame = new Mat();
        if (getCameraStream() instanceof WebcamStream)
            bgr.copyTo(frame);
        else
            Core.rotate(bgr, frame, Core.ROTATE_90_COUNTERCLOCKWISE);
        int y = text(frame, state, 0, 10);
        text(frame, Utils.elapsedTime(System.currentTimeMillis() - start), 0, y);
        video.put(frame);
        frame.release();
        return bgr;
    }

    private int text(Mat m, String text, int x, int y)
    {
        int[] base = new int[1];
        Size textSize = Imgproc.getTextSize(text, Imgproc.FONT_HERSHEY_PLAIN, 1, 1, base);
        Rect r = new Rect(x, y, (int)textSize.width, (int)textSize.height);
        Imgproc.rectangle(m, r, new Scalar(0, 0, 0), -1);
        Imgproc.putText(m, text, new Point(x, y + base[0]), Imgproc.FONT_HERSHEY_PLAIN, 1, new Scalar(255, 255, 255));
        return base[0] + y + 12;
    }

}
