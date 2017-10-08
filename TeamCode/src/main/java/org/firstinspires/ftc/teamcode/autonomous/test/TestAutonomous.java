package org.firstinspires.ftc.teamcode.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskClassifyPictograph;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskDelay;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskReadSensor;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskRunServo;
import org.firstinspires.ftc.teamcode.autonomous.util.TelemetryWrapper;
import org.firstinspires.ftc.teamcode.autonomous.util.opencv.PictographFinder;

/**
 * An example test autonomous OpMode. Test OpModes should perhaps be @Disabled at competitions
 * to declutter?
 */
@Autonomous(name = "Test Autonomous", group="test")
@SuppressWarnings("unused")
public class TestAutonomous extends BaseAutonomous {

    private Servo servo;
    private TaskClassifyPictograph finder;

    @Override
    public void initialize() throws InterruptedException {
        //Must do for ALL servos!!
        servo = hardwareMap.servo.get("s0");
        servo.setPosition(-0.1);
        finder = new TaskClassifyPictograph();
    }

    @Override
    public void run() throws InterruptedException {
        //Put tasks here, like this:
        //tasks.add(new TaskReadSensor(hardwareMap.colorSensor.get("color_sensor")));
        tasks.add(finder);
        runTasks();
        TelemetryWrapper.setLines(1);
        TelemetryWrapper.setLine(0, "Classification Result: " + finder.getResult());
        //If you need to run the tasks before continuing, put:
        //runTasks();
    }
}
