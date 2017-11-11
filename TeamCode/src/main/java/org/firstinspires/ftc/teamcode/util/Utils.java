package org.firstinspires.ftc.teamcode.util;

import android.util.Log;

import org.firstinspires.ftc.teamcode.autonomous.util.Config;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.zip.GZIPOutputStream;

/**
 * Utils - Utility methods
 */

public class Utils {
    /**
     * Return a string representation of a double with 4-digit precision.
     * @param r The number
     * @return A string representing the double to 4 digits of precision
     */
    public static String shortFloat(double r) {
        return String.format("%.4f", r);
    }
    /**
     * Constrain a value between a lower and upper limit. Good for limiting servo values etc.
     * @param value The value to constrain
     * @param min   The lower limit
     * @param max   The upper limit
     * @return value if it is between min and max, otherwise if it is less than min returns min and
     *               if greater than max returns max.
     */
    public static double constrain(double value, double min, double max) {
        return (value < min) ? min : ((value > max) ? max : value);
    }

    public static double scaleRange(double x, double a, double b, double c, double d) {
        return c * (1.0 - (x - a) / (b - a)) + d * ((x - a) / (b - a));
    }

    /**
     * GZips all non-GZipped old Robot Controller log files, renaming them to
     * 'robotControllerLog.txt.[n].gz'
     */
    public static void gzipLogs() {
        String base = Config.baseDir + "/robotControllerLog.txt.";
        File input;
        for (int i = 1; (input = new File(base + i)).exists(); i++) {
            try (InputStream inp = new FileInputStream(input);
                 OutputStream out = new GZIPOutputStream(
                         new FileOutputStream(input.getAbsolutePath() + ".gz"))) {
                byte[] buffer = new byte[1024];
                int r;
                while ((r = inp.read(buffer)) > 0) {
                    out.write(buffer, 0, r);
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
            input.delete();
            Log.i("LogZipper", "GZipped log #" + i);
        }
    }
}
