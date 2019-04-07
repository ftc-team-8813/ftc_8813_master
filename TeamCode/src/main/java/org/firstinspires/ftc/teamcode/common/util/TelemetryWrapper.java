package org.firstinspires.ftc.teamcode.common.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * TelemetryWrapper - Makes the telemetry readouts a bit more readable and manageable.
 */

public class TelemetryWrapper
{
    private static Telemetry t;
    private static Telemetry.Item[] lines;
    
    public static void init(Telemetry t, int nlines)
    {
        t.clear();
        TelemetryWrapper.t = t;
        lines = new Telemetry.Item[nlines];
        init();
    }

    private static void render()
    {
        t.update();
    }

    private static void init()
    {
        t.setCaptionValueSeparator("");
        for (int i = 0; i < lines.length; i++)
        {
            lines[i] = t.addData("", "");
        }
        render();
    }
    
    public static synchronized void setLine(int l, String message)
    {
        if (l < 0 || l >= lines.length) return;
        lines[l].setValue(message);
        render();
    }
    
    public static void setLines(int l)
    {
        t.clear();
        lines = new Telemetry.Item[l];
        init();
    }
    
    public static void clear()
    {
        t.setCaptionValueSeparator(": ");
        for (int i = 0; i < lines.length; i++)
        {
            t.removeItem(lines[i]);
        }
        render();
    }
}
