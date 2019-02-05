package org.firstinspires.ftc.teamcode.autonomous.test.opencv;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskSample;
import org.firstinspires.ftc.teamcode.autonomous.util.opencv.CameraStream;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.sensors.vision.ShapeGoldDetector;

@Autonomous(name="OpenCV test")
public class OpenCVTest extends BaseAutonomous
{
    private ShapeGoldDetector detector;
    private static final int w = 640, h = 480;

    /*
    Vision Coordinate System:

    (phone upright)
                ^ -X   *
                |       (0,0)
                |
         <------+------>
         +Y     |      -Y
                |
                v +X
     */
    @Override
    public void run() throws InterruptedException
    {
        Robot robot = Robot.instance();
        CameraStream stream = getCameraStream();
        detector = new ShapeGoldDetector();
        stream.addModifier(detector);
        stream.addListener(detector);

        DcMotor left = robot.leftFront;
        DcMotor right = robot.rightFront;
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotor lf = robot.leftFront;
        DcMotor rf = robot.rightFront;
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        new TaskSample(detector, true).runTask();
    }
}
