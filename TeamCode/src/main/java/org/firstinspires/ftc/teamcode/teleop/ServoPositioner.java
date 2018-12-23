package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.util.telemetry.TelemetryWrapper;
import org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper;
import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.firstinspires.ftc.teamcode.common.util.Utils;

import java.io.File;
import java.io.IOException;
import java.util.Map;
import java.util.Set;

import static org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper.b;
import static org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper.dpad_down;
import static org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper.dpad_up;

/**
 * ServoPositioner - Tool to move specific servos individually to collect positions
 */

@TeleOp(name = "Servo Positioner")
public class ServoPositioner extends OpMode
{
    
    private static final int STATE_CHOOSING = 1;
    private static final int STATE_RUNNING = 2;
    private boolean init_draw = true;
    private int state;
    private String servo;
    private String[] servos;
    private Servo s;
    private int chosen;
    private int scroll;
    private double value;
    private static final int VISIBLE_LINES = 3;
    private ButtonHelper helper;
    
    
    @Override
    public void init()
    {
        try
        {
            Logger.init(new File(Config.storageDir + "latest.log"));
        } catch (IOException e)
        {
            throw new RuntimeException(e);
        }
        state = STATE_CHOOSING;
        helper = new ButtonHelper(gamepad1);
        init_draw = true;
    }
    
    @Override
    public void loop()
    {
        switch (state)
        {
            case STATE_CHOOSING:
                if (servos == null)
                {
                    servos = keys(hardwareMap.servo.entrySet());
                    TelemetryWrapper.init(telemetry, VISIBLE_LINES + 1);
                    TelemetryWrapper.setLine(0, "Choose a servo to move; press B to select");
                } else
                {
                    if (init_draw)
                    {
                        init_draw = false;
                        draw();
                    }
                    if (helper.pressing(dpad_up) && chosen > 0)
                    {
                        chosen--;
                        if (chosen < scroll) scroll = chosen;
                        draw();
                    }
                    if (helper.pressing(dpad_down) && chosen < servos.length - 1)
                    {
                        chosen++;
                        if (chosen >= scroll + VISIBLE_LINES) scroll = chosen - VISIBLE_LINES + 1;
                        draw();
                    }
                    if (helper.pressing(b))
                    {
                        servo = servos[chosen];
                        state = STATE_RUNNING;
                    }
                }
                break;
            case STATE_RUNNING:
                if (s == null)
                {
                    TelemetryWrapper.setLines(2);
                    TelemetryWrapper.setLine(0, "Use the left joystick up/down to control the servo");
                    s = hardwareMap.servo.get(servo);
                }
                value += -0.02 * gamepad1.left_stick_y;
                value = Utils.constrain(value, 0, 1);
                s.setPosition(value);
                TelemetryWrapper.setLine(1, "Value: " + value);
                try
                {
                    Thread.sleep(50);
                } catch (InterruptedException e)
                {
                    return;
                }
                break;
            default:
                break;
        }
    }
    
    @Override
    public void stop()
    {
        Logger.close();
    }
    
    private void draw()
    {
        for (int i = 0; i < VISIBLE_LINES; i++)
        {
            if (scroll + i >= servos.length) break;
            String s = servos[scroll + i];
            if (scroll + i == chosen)
            {
                s = "> " + s;
            }
            TelemetryWrapper.setLine(i + 1, s);
        }
    }
    
    @SuppressWarnings("unchecked")
    private String[] keys(Set<Map.Entry<String, Servo>> set)
    {
        String[] out = new String[set.size()];
        Object[] entries = set.toArray();
        for (int i = 0; i < set.size(); i++)
        {
            out[i] = ((Map.Entry<String, ?>) entries[i]).getKey();
        }
        return out;
    }
}
