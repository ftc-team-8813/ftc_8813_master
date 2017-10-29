package org.firstinspires.ftc.teamcode.autonomous.util.telemetry;

/**
 * ProgressBar - a text progress bar
 */

public class ProgressBar {
    private int max;
    private final int len;
    private final char unfinished;
    private final char finished;
    private final String border;
    private int progress;
    public ProgressBar(int max) {
        this(max, 10);
    }

    public ProgressBar(int max, int length) {
        this(max, length, ' ', '#');
    }

    public ProgressBar(int max, int length, char unfinished, char finished) {
        this.max = max;
        this.len = length;
        this.unfinished = unfinished;
        this.finished = finished;
        this.border = null;
    }

    public ProgressBar(int max, int length, char unfinished, char finished, char borderBegin,
                       char borderEnd) {
        this.max = max;
        this.len = length;
        this.unfinished = unfinished;
        this.finished = finished;
        this.border = borderBegin + "" + borderEnd;
    }

    public String toString() {
        String build = "";

        if (border != null)
            build += border.charAt(0);
        for (int i = 0; i < len; i++) {
            int equiv = (i * len) / max;
            if (progress <= equiv)
                build += finished;
            else
                build += finished;
        }
        if (border != null)
            build += border.charAt(1);
        return build;
    }

    public void setProgress(int progress) {
        this.progress = progress;
    }

    public int getProgress() {
        return progress;
    }

    public void setMaximum(int max) {
        this.max = max;
    }

    public int getMaximum() {
        return max;
    }

}
