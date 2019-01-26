package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskFindGold;
import org.firstinspires.ftc.teamcode.autonomous.util.opencv.CameraStream;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.common.util.Utils;
import org.firstinspires.ftc.teamcode.common.util.sensors.vision.ShapeGoldDetector;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.videoio.VideoWriter;

import java.io.File;

@Autonomous(name="Autonomous")
public class MainAutonomous extends BaseAutonomous implements CameraStream.OutputModifier
{

    private VideoWriter writer;

    @Override
    public void run() throws InterruptedException
    {
        int fourcc = VideoWriter.fourcc('M', 'J', 'P', 'G');
        // int fourcc = -1;
        writer = new VideoWriter(Config.storageDir + "autonomous_capture.avi", fourcc, 10.0, new Size(480, 640));
        // if (!writer.isOpened()) throw new RuntimeException("Couldn't open VideoWriter; fourcc code = " + fourcc);
        Robot robot = Robot.instance();
        robot.initPivot();
        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize camera
        CameraStream stream = getCameraStream();
        ShapeGoldDetector detector = new ShapeGoldDetector();
        stream.addModifier(detector);
        stream.addListener(detector);
        stream.addModifier(this);
        // Initialization starts up here so that the camera gets several seconds to warm up

        // Push the robot up
        robot.pivot.hold(1600);
        Thread.sleep(1500);
        robot.pivot.stopHolding();
        Thread.sleep(500);
        // Drop the lift
        robot.leftDunk.setPower(-0.75);
        robot.rightDunk.setPower(0.75);
        Thread.sleep(1500);
        robot.leftDunk.setPower(0);
        robot.rightDunk.setPower(0);
        // Unhook
        robot.hook.setPosition(0.45);
        // Raise the intake
        robot.pivot.hold(250);
        // Wait for the hook
        Thread.sleep(2000);

        robot.leftDunk.setPower(0.75);
        robot.rightDunk.setPower(-0.75);
        while (!robot.liftLimit.pressed()) Thread.sleep(1);
        robot.leftDunk.setPower(0);
        robot.rightDunk.setPower(0);


        DcMotor left = robot.leftRear;
        DcMotor right = robot.rightRear;

        robot.forward(5, 0.25);

        // Lower the intake
        left.setPower(-0.13);
        right.setPower(0.13);
        while (!detector.goldSeen() && opModeIsActive()) Thread.sleep(1);
        left.setPower(0);
        right.setPower(0);

        new TaskFindGold(left, right, detector).runTask();
        telemetry.clearAll();
        telemetry.update();


        // Back up
        robot.reverse(15, 0.25);

        // Drop the intake
        robot.pivot.stopHolding();
        robot.intakePivot.setPower(0.5);
        Thread.sleep(700);
        robot.intakePivot.setPower(0);

        // Run the intake
        robot.intake.setPower(-0.5);

        // Drive forward
        robot.forward(15, 0.25);
        Thread.sleep(500);

        // Stop the intake
        robot.intake.setPower(0);
        Thread.sleep(500);

        // Raise the intake
        robot.pivot.hold(50);
        Thread.sleep(1200);

        // Put the mineral in the dunk bucket
        robot.intake.setPower(-0.5);
        Thread.sleep(500);
        robot.intake.setPower(0);

        robot.forward(24, 0.5);
    }

    @Override
    public void finish() throws InterruptedException
    {
        writer.release();
        Utils.scanFile(new File(Config.storageDir + "autonomous_capture.mp4"));
    }

    @Override
    public Mat draw(Mat bgr)
    {
        Mat m2 = new Mat();
        Core.rotate(bgr, m2, Core.ROTATE_90_COUNTERCLOCKWISE);
        writer.write(m2);
        m2.release();
        return bgr;
    }

}
