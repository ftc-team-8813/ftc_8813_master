package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskFindGold;
import org.firstinspires.ftc.teamcode.autonomous.util.opencv.CameraStream;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.sensors.vision.ShapeGoldDetector;

@Autonomous(name="Autonomous")
public class MainAutonomous extends BaseAutonomous
{
    @Override
    public void run() throws InterruptedException
    {
        Robot robot = Robot.instance();
        robot.initPivot();
        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        robot.leftDunk.setPower(-1);
        robot.rightDunk.setPower(1);
        robot.leftRear.setPower(0.5);
        robot.rightRear.setPower(0.5);
        long startTime = System.currentTimeMillis();
        int start = robot.leftRear.getCurrentPosition();
        while (robot.leftRear.getCurrentPosition() - start < Robot.ENC_PER_ROTATION_20) Thread.sleep(5);
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);
        Thread.sleep(1000 - (System.currentTimeMillis() - start)); // Who needs acccuracy?
        robot.leftDunk.setPower(0);
        robot.rightDunk.setPower(0);
        robot.hook.setPosition(0);
        Thread.sleep(3000); // Wait for the SLOOOOOW servo
        // We now have 24 seconds

        DcMotor left = robot.leftRear;
        DcMotor right = robot.rightRear;

        // Initialize camera
        CameraStream stream = getCameraStream();
        ShapeGoldDetector detector = new ShapeGoldDetector();
        stream.addModifier(detector);
        stream.addListener(detector);

        left.setPower(-0.2);
        right.setPower(0.2);
        while (!detector.goldSeen() && opModeIsActive()) Thread.sleep(5);
        left.setPower(0);
        right.setPower(0);

        new TaskFindGold(left, right, detector).runTask();
        telemetry.clearAll();
        telemetry.update();
    }
}
