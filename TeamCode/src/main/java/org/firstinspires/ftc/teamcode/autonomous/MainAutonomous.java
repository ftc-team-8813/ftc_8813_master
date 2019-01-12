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
        Robot robot = Robot.initialize(hardwareMap, config);
        robot.initPivot();
        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        DcMotor left = robot.leftRear;
        DcMotor right = robot.rightRear;

        // Initialize camera
        CameraStream stream = getCameraStream();
        ShapeGoldDetector detector = new ShapeGoldDetector();
        stream.addModifier(detector);
        stream.addListener(detector);

        left.setPower(-0.1);
        right.setPower(0.1);
        while (!detector.goldSeen() && opModeIsActive()) Thread.sleep(5);
        left.setPower(0);
        right.setPower(0);

        right.setDirection(DcMotorSimple.Direction.FORWARD);
        new TaskFindGold(left, right, detector).runTask();
        telemetry.clearAll();
        telemetry.update();
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setPower(0.5);
        right.setPower(0.5);
        Thread.sleep(1000);
        left.setPower(0);
        right.setPower(0);
    }
}
