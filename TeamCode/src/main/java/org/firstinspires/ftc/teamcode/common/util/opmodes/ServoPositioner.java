package org.firstinspires.ftc.teamcode.common.util.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.util.Chooser;
import org.firstinspires.ftc.teamcode.common.util.TelemetryWrapper;
import org.firstinspires.ftc.teamcode.common.util.DataStorage;
import org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper;
import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.firstinspires.ftc.teamcode.common.util.Utils;

import java.io.File;
import java.io.IOException;

import static org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper.a;
import static org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper.b;
import static org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper.dpad_down;
import static org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper.dpad_up;
import static org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper.left_bumper;
import static org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper.x;

/**
 * ServoPositioner - Tool to oldMove specific servos individually to collect positions
 */

@TeleOp(name = "Servo Positioner")
public class ServoPositioner extends OpMode
{
    
    private static final int STATE_CHOOSING = 1;
    private static final int STATE_RUNNING = 2;
    private Chooser chooser;
    private int state;
    private String servo;
    private String[] servos;
    private Servo s;
    private double value;
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
                    chooser = new Chooser("Choose a servo to oldMove; press B to select", servos, helper, telemetry);
                } else
                {
                    chooser.update();
                    if (chooser.chosen())
                    {
                        servo = (String)chooser.getSelected();
                        state = STATE_RUNNING;
                    }
                }
                break;
            case STATE_RUNNING:
                if (s == null)
                {
                    TelemetryWrapper.setLines(6);
                    TelemetryWrapper.setLine(0, "Use the left joystick up/down to control the servo");
                    TelemetryWrapper.setLine(1, "Use the dpad up/down to choose a preset");
                    TelemetryWrapper.setLine(2, "Press A to set the preset and X to oldMove to it");
                    TelemetryWrapper.setLine(3, "Press LEFT BUMPER to select a different servo");
                    s = hardwareMap.servo.get(servo);
                    servoPositions = new DataStorage(new File(Config.storageDir + "servo_positions.json"));
                    positionNames = presetNames.getStringArray(servo + "_presets");
                    if (positionNames != null)
                    {
                        for (int i = 0; i < positionNames.length; i++)
                            positionNames[i] = servo + "." + positionNames[i];
                        presetValue = servoPositions.getDouble(positionNames[selectedPreset], 0);
                        value = presetValue;
                    }
                    else
                    {
                        positionNames = new String[] {""};
                    }
                }
                value += -0.05 * gamepad1.left_stick_y;
                value = Utils.constrain(value, 0, 1);
                s.setPosition(value);

                TelemetryWrapper.setLine(4, "Value: " + value);
                TelemetryWrapper.setLine(5, "Preset '" + positionNames[selectedPreset] + "': " + presetValue);

                if (positionNames[selectedPreset].length() > 0)
                {
                    if (helper.pressing(dpad_up))
                    {
                        selectedPreset--;
                        if (selectedPreset < 0) selectedPreset += positionNames.length;
                        presetValue = servoPositions.getDouble(positionNames[selectedPreset], value);
                        value = presetValue;
                    } else if (helper.pressing(dpad_down))
                    {
                        selectedPreset = (selectedPreset + 1) % positionNames.length;
                        presetValue = servoPositions.getDouble(positionNames[selectedPreset], value);
                        value = presetValue;
                    }

                    if (helper.pressing(a))
                    {
                        presetValue = value;
                        servoPositions.addNumber(positionNames[selectedPreset], value);
                    } else if (helper.pressing(x))
                    {
                        value = presetValue;
                    }
                }

                if (helper.pressing(left_bumper))
                {
                    servoPositions.save();
                    s = null;
                    state = STATE_CHOOSING;
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
        if (servoPositions != null) servoPositions.save();
        Logger.close();
    }
}
