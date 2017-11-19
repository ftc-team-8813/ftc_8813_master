package org.firstinspires.ftc.teamcode.autonomous.tasks;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.util.Config;
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

    public TaskDoMove(String file) {
        moves = new ArrayList<>();
        try {
            DataInputStream inp = new DataInputStream(new FileInputStream(Config.storageDir + file));
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

    }

    @Override
    public void runTask() throws InterruptedException {
        boolean rExtSet = false;
        for (RobotMove move : moves) {
            if (move.extSet) {
                rExtSet = true;
            }
            if (rExtSet && !lm.isPressed()) {
                ex.setPower(move.extendPower);
            }
            if (lm.isPressed())
                ex.setPower(0);
            ax.setPosition(move.xTurn);
            ay.setPosition(move.yTurn);
            el.setPosition(move.elbow);
            cw.setPosition(move.clawPos);
            rt.setPower(move.basePower);
            Thread.sleep(move.dt); //Should exit if robot stops
        }
    }
}
