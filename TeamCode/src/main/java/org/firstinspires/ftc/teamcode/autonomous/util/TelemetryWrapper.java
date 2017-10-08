package org.firstinspires.ftc.teamcode.autonomous.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * TelemetryWrapper - Makes the telemetry readouts a bit more readable and manageable.
 */

public class TelemetryWrapper {
    private static Telemetry t;
    private static String[] lines;
    public static void init(Telemetry t, int nlines) {
        t.clear();
        TelemetryWrapper.t = t;
        lines = new String[nlines];
        render();
    }

    private static void render() {
        for (int i = 0; i < lines.length; i++) {
            if (lines[i] == null)
                lines[i] = "";
            t.addData("" + i, lines[i]);
        }
        t.update();
    }

    public static void setLine(int l, String message) {
        if (l < 0 || l >= lines.length) return;
        lines[l] = message;
        render();
    }

    public static void setLines(int l) {
        t.clear();
        lines = new String[l];
        render();
    }

    public static void clear() {
        for (int i = 0; i < lines.length; i++) {
            lines[i] = "";
        }
    }
}
