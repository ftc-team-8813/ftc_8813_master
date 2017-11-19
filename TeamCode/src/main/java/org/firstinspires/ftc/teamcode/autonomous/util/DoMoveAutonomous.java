package org.firstinspires.ftc.teamcode.autonomous.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskDelay;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskRunServo;
import org.firstinspires.ftc.teamcode.util.Config;

/**
 * Do a single movement of the waist, shoulder and elbow servos.
 * Use to time movements.
 */

@Autonomous(name="Test Moves")
public class DoMoveAutonomous extends BaseAutonomous {
    @Override
    public void run() throws InterruptedException {
        Servo w = hardwareMap.servo.get("s0");
        Servo s = hardwareMap.servo.get("s1");
        Servo e = hardwareMap.servo.get("s2");
        Servo claw = hardwareMap.servo.get("s3");
        Config c = config;
        tasks.add(new TaskRunServo(claw, c.getDouble("claw_closed",0)));

        tasks.add(new TaskRunServo(w, c.getDouble("waist",0)));
        tasks.add(new TaskRunServo(s, c.getDouble("shoulder",0)));
        tasks.add(new TaskRunServo(e, c.getDouble("elbow",0)));

        tasks.add(new TaskDelay(3000));

        tasks.add(new TaskRunServo(w, c.getDouble("waist2",0)));
        tasks.add(new TaskRunServo(s, c.getDouble("shoulder2",0)));
        tasks.add(new TaskRunServo(e, c.getDouble("elbow2",0)));

        tasks.add(new TaskDelay(5000));

        tasks.add(new TaskRunServo(claw, c.getDouble("claw_open",0)));

        tasks.add(new TaskDelay(200));

        tasks.add(new TaskRunServo(w, c.getDouble("waist3",0)));
        tasks.add(new TaskRunServo(s, c.getDouble("shoulder3",0)));
        tasks.add(new TaskRunServo(e, c.getDouble("elbow3",0)));

        tasks.add(new TaskDelay(5000));
    }
}
