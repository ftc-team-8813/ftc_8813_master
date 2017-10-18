package org.firstinspires.ftc.teamcode.autonomous.tasks;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.util.arm.RobotMove;

import java.io.DataInputStream;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * TaskDoMove - Execute a move from a file
 */

public class TaskDoMove implements Task {

    private List<RobotMove> moves;
    private boolean loaded;
    private Servo ax, ay, el, cw;
    private DcMotor rt, ex;
    private TouchSensor lm;
    private double axt, ayt, elt, clt;
    private int rtenc, exenc;

    public TaskDoMove(String file) {
        moves = new ArrayList<>();
        try {
            DataInputStream inp = new DataInputStream(new FileInputStream(file));
            int size = inp.readInt();
            for (int i = 0; i < size; i++) {
                RobotMove next = new RobotMove();
                next.read(inp);
                moves.add(next);
            }
            inp.close();
        } catch (IOException e) {
            e.printStackTrace();
            loaded = false;
            return;
        }
        HardwareMap m = BaseAutonomous.instance().hardwareMap;
        ax = m.servo.get("s0");
        ay = m.servo.get("s1");
        el = m.servo.get("s2");
        cw = m.servo.get("s3");
        rt = m.dcMotor.get("base");
        ex = m.dcMotor.get("extend");
        lm = m.touchSensor.get("ext_bumper");
        double vel = 0.5;
        long t = System.currentTimeMillis();
        while (!lm.isPressed()) {
            ex.setPower(vel);
            if (System.currentTimeMillis() > t + 50) {
                t = System.currentTimeMillis();
                vel *= 0.75;
            }
        }
        ex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void runTask() throws InterruptedException {
        if (!loaded)
            return;
        int rEncMin = 0;
        boolean rExtSet = false;
        for (RobotMove move : moves) {
            if (move.extSet) {
                rEncMin = move.extendEncoder;
                rExtSet = true;
            }
            if (rExtSet) {
                int targetEnc = move.extendEncoder - rEncMin;
                ex.setTargetPosition(targetEnc);
                ex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
    }
}
