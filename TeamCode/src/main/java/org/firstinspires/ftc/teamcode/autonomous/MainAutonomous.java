package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskBasicSample;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskDetectGold;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskDrop;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskDunkMarker;
import org.firstinspires.ftc.teamcode.common.sensors.vision.CameraStream;
import org.firstinspires.ftc.teamcode.common.sensors.vision.WebcamStream;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.firstinspires.ftc.teamcode.common.util.Profiler;
import org.firstinspires.ftc.teamcode.common.util.Utils;
import org.firstinspires.ftc.teamcode.common.util.Vlogger;
import org.firstinspires.ftc.teamcode.autonomous.util.opencv.ShapeGoldDetector;
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
    private static final String[] sides = {"Left", "Center", "Right"};

    private int side;

    private Profiler profiler = new Profiler();

    private static final boolean DROP = true;
    private static final boolean DROP_WAIT = false;

    @Override
    public void initialize() throws InterruptedException
    {
        Robot robot = Robot.instance();
        if (DROP) robot.hook.setPosition(robot.HOOK_CLOSED);
        else robot.hook.setPosition(robot.HOOK_OPEN);
        robot.imu.initialize(telemetry);
        robot.imu.start();
        CameraStream stream = getCameraStream();
        video = new Vlogger(getVlogName(),
                (int)stream.getSize().width, (int)stream.getSize().height, 10.0);
        log = new Logger("Crater Autonomous");
        robot.mark.setPosition(0);
    }

    private String getVlogName()
    {
        int i;
        new File(Config.storageDir + "videos/").mkdir();
        for (i = 0; new File(Config.storageDir + "videos/autonomous_capture" + i + ".avi").exists(); i++);
        return "videos/autonomous_capture" + i + ".avi";
    }

    @Override
    public void run() throws InterruptedException
    {
        profiler.start("run");
        start = System.currentTimeMillis();
        Robot robot = Robot.instance();
        state = "Initializing";

        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize camera
        profiler.start("init camera");
        CameraStream stream = getCameraStream();
        ShapeGoldDetector detector = new ShapeGoldDetector();
        profiler.end();

        profiler.start("drop");
        if (DROP) new TaskDrop().runTask();
        else if (DROP_WAIT) Thread.sleep(4000);
        robot.imu.resetHeading();
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
        new TaskBasicSample(detector).runTask();
        profiler.end();
        telemetry.clearAll();
        telemetry.update();

        robot.imu.update();
        if (robot.imu.getHeading() >= 25) side = LEFT;
        else if (robot.imu.getHeading() <= -25) side = RIGHT;
        else side = CENTER;
        telemetry.addData("Side", sides[side + 1]).setRetained(true);

        state = "Drive to depot";
        profiler.start("depot");
        profiler.start("reset_turn");
        robot.turnTo(0, 0.3);
        profiler.start("forward");
        robot.forward(25, 0.3);
        profiler.end();
        profiler.start("turn");
        robot.turnTo(-105, 0.3);
        profiler.end();

        profiler.start("curve");

        robot.reverse(80, 0.7);
        robot.turnTo(-45, 0.3);

        profiler.start("dunk");
        robot.reverse(55, 0.6);
        new TaskDunkMarker().runTask();
        profiler.end(); // depot

        state = "Park in crater";
        profiler.start("park");
        robot.forward(90, 1.0);
        robot.intakeExtController.hold(robot.ext_max / 2);
        profiler.end();
        profiler.end(); // run

    }

    @Override
    public synchronized void finish()
    {
        video.close();
        log.d("Crater autonomous finished -- mineral=%s, drop=%s", sides[side+1], Boolean.toString(DROP));
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
        // bgr is passed through; the image is not modified
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
