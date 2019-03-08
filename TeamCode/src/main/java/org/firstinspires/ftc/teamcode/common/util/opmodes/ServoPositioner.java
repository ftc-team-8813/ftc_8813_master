package org.firstinspires.ftc.teamcode.common.util.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.util.telemetry.TelemetryWrapper;
import org.firstinspires.ftc.teamcode.common.util.DataStorage;
import org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper;
import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.firstinspires.ftc.teamcode.common.util.Utils;

import java.io.File;
import java.io.IOException;
import java.util.Map;
import java.util.Set;

import static org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper.a;
import static org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper.b;
import static org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper.back;
import static org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper.dpad_down;
import static org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper.dpad_up;
import static org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper.x;

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
    private static final int VISIBLE_LINES = 5;
    private ButtonHelper helper;
    private Config presetNames;
    private DataStorage servoPositions;
    private String[] positionNames;
    private int selectedPreset = 0;
    private double presetValue = 0;
    
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
        presetNames = new Config("preset_names.txt");
    }
    
    @Override
    public void loop()
    {
        switch (state)
        {
            case STATE_CHOOSING:
                if (servos == null)
                {
                    servos = Utils.allDeviceNames(hardwareMap.servo);
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
                    TelemetryWrapper.setLines(5);
                    TelemetryWrapper.setLine(0, "Use the left joystick up/down to control the servo");
                    TelemetryWrapper.setLine(1, "Use the dpad up/down to choose a preset; press A to set the preset and X to move to it");
                    TelemetryWrapper.setLine(2, "Press BACK to select a different servo");
                    s = hardwareMap.servo.get(servo);
                    servoPositions = new DataStorage(new File(Config.storageDir + "servo_positions.json"));
                    positionNames = presetNames.getStringArray(servo + "_presets");
                    for (int i = 0; i < positionNames.length; i++) positionNames[i] = servo + "." + positionNames[i];
                    presetValue = servoPositions.getDouble(positionNames[selectedPreset], 0);
                    value = presetValue;
                }
                value += -0.05 * gamepad1.left_stick_y;
                value = Utils.constrain(value, 0, 1);
                s.setPosition(value);

                TelemetryWrapper.setLine(3, "Value: " + value);
                TelemetryWrapper.setLine(4, "Preset '" + positionNames[selectedPreset] + "': " + presetValue);

                if (helper.pressing(dpad_up))
                {
                    selectedPreset--;
                    if (selectedPreset < 0) selectedPreset += positionNames.length;
                    presetValue = servoPositions.getDouble(positionNames[selectedPreset], value);
                    value = presetValue;
                }
                else if (helper.pressing(dpad_down))
                {
                    selectedPreset = (selectedPreset + 1) % positionNames.length;
                    presetValue = servoPositions.getDouble(positionNames[selectedPreset], value);
                    value = presetValue;
                }

                if (helper.pressing(a))
                {
                    presetValue = value;
                    servoPositions.addNumber(positionNames[selectedPreset], value);
                }
                else if (helper.pressing(x))
                {
                    value = presetValue;
                }

                if (helper.pressing(back))
                {
                    servoPositions.save();
                    s = null;
                    state = STATE_CHOOSING;
                    init_draw = true;
                    servos = null;
                }

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
        servoPositions.save();
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
}
