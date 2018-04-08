package org.firstinspires.ftc.teamcode.util;

import android.media.MediaScannerConnection;
import android.net.Uri;
import android.util.Log;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.text.SimpleDateFormat;
import java.util.zip.GZIPOutputStream;

/**
 * Utils - Utility methods
 */

public class Utils {
    /**
     * Return a time in elapsed-time format (hh:mm:ss:hundredths)
     * @param millis The time to format
     * @return An elapsed-time string
     */
    public static String elapsedTime(long millis) {
        int hundredths = (int)((millis/10)%100);
        int seconds    = (int)((millis/1000)%60);
        int minutes    = (int)((millis/60000)%60);
        int hours      = (int)((millis/3600000));
        return String.format("%02d:%02d:%02d.%03d", hours, minutes, seconds, hundredths);
    }

    public static void scanFile(File file) {
        MediaScannerConnection.scanFile(AppUtil.getDefContext(), new String[] { file.toString() }, null,
                new MediaScannerConnection.OnScanCompletedListener() {
                    public void onScanCompleted(String path, Uri uri) {
                        Log.i("ExternalStorage", "Scanned " + path + ":");
                        Log.i("ExternalStorage", "-> uri=" + uri);
                    }
                });
    }

    public static double mean(double... values) {
        return sum(values) / values.length;
    }

    public static double sum(double... values) {
        double total = 0;
        for (double d : values) {
            total += d;
        }
        return total;
    }
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

    /**
     * Creates a scaling factor which scales the range [a,b] to the range [c,d] and applies it to x. <br>
     * For example,
     * <pre>
     *     scaleRange(x, 0, 1, 0, 2) -> x * 2
     *     scaleRange(x, 0, 1, 1, 2) -> x + 1
     *     scaleRange(x, 0, pi*2, 0, 360) -> convert x from radians to degrees
     *     scaleRange(x, 0, 1, 1, 0) -> 1 - x
     *     scaleRange(x, 0, 1, 1, 3) -> 2*x + 1
     * </pre>
     *
     * @param x The value to scale
     * @param a The minimum of the first range
     * @param b The maximum of the first range
     * @param c The minimum of the second range
     * @param d The maximum of the second range
     * @return
     */
    public static double scaleRange(double x, double a, double b, double c, double d) {
        double o = c * (1.0 - (x - a) / (b - a)) + d * ((x - a) / (b - a));
        if (o == Double.NaN) {
            new Logger("Utils.scaleRange()").e("NaN when scaling %.2f-%.2f to %.2f-%.2f");
        }
        return o;
    }

    public static String quadName(int quadrant) {
        switch (quadrant) {
            case 1: return "Blue Upper";
            case 2: return "Blue Lower";
            case 3: return "Red Lower";
            case 4: return "Red Upper";
            default: return "Unknown";
        }
    }

    //Useless function
//    /**
//     * GZips all non-GZipped old Robot Controller log files, renaming them to
//     * 'robotControllerLog.txt.[n].gz'
//     */
//    public static void gzipLogs() {
//        String base = Config.baseDir + "/robotControllerLog.txt.";
//        File input;
//        for (int i = 1; (input = new File(base + i)).exists(); i++) {
//            try (InputStream inp = new FileInputStream(input);
//                 OutputStream out = new GZIPOutputStream(
//                         new FileOutputStream(input.getAbsolutePath() + ".gz"))) {
//                byte[] buffer = new byte[1024];
//                int r;
//                while ((r = inp.read(buffer)) > 0) {
//                    out.write(buffer, 0, r);
//                }
//            } catch (IOException e) {
//                e.printStackTrace();
//            }
//            input.delete();
//            Log.i("LogZipper", "GZipped log #" + i);
//        }
//    }
}
