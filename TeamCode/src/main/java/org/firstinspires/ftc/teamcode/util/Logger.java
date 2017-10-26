package org.firstinspires.ftc.teamcode.util;

import java.io.File;
import java.io.OutputStream;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.List;

/**
 * Created by aidan on 10/25/17.
 */

public class Logger {
    private final String name;
    private static PrintStream[] outputs;
    public static enum Level {
        TRACE(10), DEBUG(20), INFO(30), WARN(40), ERROR(50), FATAL(100000)
        ;
        private int l;
        Level(int l) {
            this.l = l;
        }

        public int getLevel() {
            return l;
        }
    }
    private static int minLevel;
    public Logger(String name) {
        this.name = name;
    }

    public static Logger getLogger() {
        return new Logger(Thread.currentThread().getStackTrace()[1].getClass().getSimpleName());
    }

    public static void setOutputs(PrintStream... outs) {
        outputs = outs;
    }

    private void log(String message) {
        for (PrintStream out : outputs) {
            out.println(message);
        }
    }

    public void log(int level, Object message) {
        if (level < minLevel)
            return;

    }
}
