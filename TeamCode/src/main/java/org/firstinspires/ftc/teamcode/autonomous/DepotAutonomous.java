package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskBasicSample;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskDetectGold;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskDrop;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskDunkMarker;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskSample;
import org.firstinspires.ftc.teamcode.common.sensors.vision.CameraStream;
import org.firstinspires.ftc.teamcode.common.sensors.vision.WebcamStream;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.firstinspires.ftc.teamcode.common.util.Profiler;
import org.firstinspires.ftc.teamcode.common.util.Vlogger;
import org.firstinspires.ftc.teamcode.autonomous.util.opencv.ShapeGoldDetector;
import org.opencv.core.Core;
import org.opencv.core.Mat;

import java.io.File;

@Autonomous(name="Depot Autonomous")
public class DepotAutonomous extends BaseAutonomous implements CameraStream.OutputModifier
{
    private Vlogger vlogger;
    private static final int LEFT = -1;
    private static final int CENTER = 0;
    private static final int RIGHT = 1;

    private int side;

    private static final String[] sides = {"Left", "Center", "Right"};

    private Profiler profiler = new Profiler();
    private Logger log = new Logger("Depot Autonomous");

    protected boolean OTHER_CRATER = false;

    public static final boolean DROP = false;
    public static final boolean DROP_WAIT = true;

    @Override
    public void initialize() throws InterruptedException
    {
        Robot robot = Robot.instance();
        robot.hook.setPosition(robot.HOOK_CLOSED);
        robot.imu.initialize(telemetry);
        robot.imu.start();

        robot.initPivotAuto();
        CameraStream stream = getCameraStream();
        vlogger = new Vlogger(getVlogName(),
                (int)stream.getSize().width, (int)stream.getSize().height, 10.0);
        robot.mark.setPosition(0.7);
    }

    private String getVlogName()
    {
        int i;
        for (i = 0; new File(Config.storageDir + "videos/autonomous_capture" + i + ".avi").exists(); i++);
        return "videos/autonomous_capture" + i + ".avi";
    }

    @Override
    public void run() throws InterruptedException
    {
        profiler.start("run");
        Robot robot = Robot.instance();
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

        DcMotor left = robot.leftRear;
        DcMotor right = robot.rightRear;
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stream.addModifier(detector);
        stream.addListener(detector);
        stream.addModifier(this);

        profiler.start("detect");
        new TaskDetectGold(detector, profiler).runTask();
        profiler.end();

        profiler.start("sample");
        new TaskBasicSample(detector).runTask();
        profiler.end();
        telemetry.clearAll();
        telemetry.update();

        robot.imu.update();
        if (robot.imu.getHeading() >= 25) side = LEFT;
        else if (robot.imu.getHeading() <= -25) side = RIGHT;
        else side = CENTER;

        profiler.start("drop marker");
        profiler.start("reset turn");
        robot.turnTo(0, 0.3);
        profiler.end();

        profiler.start("forward");
        robot.forward(20, 0.5);
        profiler.end();
        profiler.start("turn");
        robot.turnTo(75, 0.5);

        profiler.start("forward2");
        robot.forward(80, 0.8);
        Thread.sleep(200);
        profiler.end();

        profiler.start("turn2");
        robot.turnTo(135, 0.5);
        profiler.end();
        profiler.start("reverse");
        robot.reverse(48, 0.8);
        profiler.end();

        profiler.start("dunk");
        new TaskDunkMarker().runTask();
        profiler.end();

        profiler.end(); // drop marker

        profiler.start("park");
        robot.forward(88, 1);
        robot.intakeExtController.hold(robot.ext_max / 2);
        profiler.end(); // park
        profiler.end(); // run
    }

    @Override
    public void finish() throws InterruptedException
    {
        vlogger.close();
        log.d("Depot Autonomous finished -- mineral=%s, drop=%s", sides[side+1], Boolean.toString(DROP));
        profiler.finish();
    }

    @Override
    public Mat draw(Mat bgr)
    {
        Mat m2 = new Mat();
        if (getCameraStream() instanceof WebcamStream)
            bgr.copyTo(m2);
        else
            Core.rotate(bgr, m2, Core.ROTATE_90_COUNTERCLOCKWISE);
        vlogger.put(m2);
        m2.release();
        return bgr;
    }

}
