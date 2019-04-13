package org.firstinspires.ftc.teamcode.common.util;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper;

public class Chooser
{
    private String prompt;
    private int selected;
    private int scroll = 0;
    private boolean chosen = false;
    private boolean draw = true;
    private Object[] values;
    private ButtonHelper buttonHelper;
    private int up_button = ButtonHelper.dpad_up;
    private int down_button = ButtonHelper.dpad_down;
    private int trigger_button = ButtonHelper.b;
    private static final int nLines = 5;

    public Chooser(String prompt, Object[] values, ButtonHelper buttonHelper, Telemetry telemetry)
    {
        this.prompt = prompt;
        this.values = values;
        TelemetryWrapper.init(telemetry, nLines + 1);
        this.buttonHelper = buttonHelper;
    }

    public Chooser(String prompt, Object[] values, Gamepad gamepad, Telemetry telemetry)
    {
        this(prompt, values, new ButtonHelper(gamepad), telemetry);
    }

    public void setEnterButton(int button)
    {
        this.trigger_button = button;
    }

    public void update()
    {
        if (chosen) return;

        if (buttonHelper.pressing(up_button))
        {
            selected--;
            if (selected < 0) selected += values.length;
            draw = true;
        }
        if (buttonHelper.pressing(down_button))
        {
            selected = (selected + 1) % values.length;
            draw = true;
        }
        if (buttonHelper.pressing(trigger_button))
        {
            chosen = true;
            TelemetryWrapper.clear();
            return;
        }
        if (draw)
        {
            draw = false;
            TelemetryWrapper.setLine(0, prompt);
            if (selected < scroll) scroll = selected;
            else if (selected >= scroll + nLines) scroll = selected - nLines + 1;
            for (int i = scroll, j = 1; i < scroll + nLines && i < values.length; i++, j++)
            {
                if (i == selected) TelemetryWrapper.setLine(j, "> " + values[i].toString());
                else TelemetryWrapper.setLine(j, "  " + values[i].toString());
            }
        }
    }

    public boolean chosen()
    {
        return chosen;
    }

    public int getSelectedIndex()
    {
        return selected;
    }

    public Object getSelected()
    {
        return values[selected];
    }

    public void choose()
    {
        if (chosen) return;
        chosen = true;
        TelemetryWrapper.clear();
    }
}
