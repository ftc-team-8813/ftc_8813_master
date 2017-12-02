package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.util.telemetry.TelemetryWrapper;

import java.lang.reflect.Array;
import java.util.Map;
import java.util.Set;

/**
 * Created by aidan on 12/1/17.
 */

public class ServoPositioner extends OpMode {

    private static final int STATE_CHOOSING = 1;
    private static final int STATE_RUNNING = 2;
    private int state;
    private String servo;
    private String[] servos;
    private int chosen;
    private int scroll;


    @Override
    public void init() {
        state = STATE_CHOOSING;
    }

    @Override
    public void loop() {
        switch (state) {
            case STATE_CHOOSING:
                if (servos == null) {
                    servos = keys(hardwareMap.servo.entrySet());
                    TelemetryWrapper.setLines(4);
                    TelemetryWrapper.setLine(0, "Choose a servo to move");
                }

                break;
            case STATE_RUNNING:

                break;
            default: break;
        }
    }

    @SuppressWarnings("unchecked")
    private String[] keys(Set<Map.Entry<String, Servo>> set) {
        String[] out = new String[set.size()];
        Object[] entries = set.toArray();
        for (int i = 0; i < set.size(); i++) {
            out[i] = ((Map.Entry<String, ?>)entries[i]).getKey();
        }
        return out;
    }
}
