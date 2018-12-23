package org.firstinspires.ftc.teamcode.common.util;

import android.media.MediaScannerConnection;
import android.net.Uri;
import android.util.Log;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Utils - Utility methods
 */

public class Utils
{
    /**
     * Return a time in elapsed-time format (hh:mm:ss:hundredths)
     *
     * @param millis The time to format
     * @return An elapsed-time string
     */
    public static String elapsedTime(long millis)
    {
        int hundredths = (int) ((millis / 10) % 100);
        int seconds = (int) ((millis / 1000) % 60);
        int minutes = (int) ((millis / 60000) % 60);
        int hours = (int) ((millis / 3600000));
        return String.format("%02d:%02d:%02d.%03d", hours, minutes, seconds, hundredths);
    }
    
    /**
     * Scan a new/deleted file so that the change is reflected in the file system when connected to
     * USB
     *
     * @param file The file to scan
     */
    public static void scanFile(File file)
    {
        MediaScannerConnection.scanFile(AppUtil.getDefContext(), new String[]{file.toString()}, null,
                new MediaScannerConnection.OnScanCompletedListener()
                {
                    public void onScanCompleted(String path, Uri uri)
                    {
                        Log.i("ExternalStorage", "Scanned " + path + ":");
                        Log.i("ExternalStorage", "-> uri=" + uri);
                    }
                });
    }
    
    /**
     * Return a string representation of a double with 4-digit precision.
     *
     * @param r The number
     * @return A string representing the double to 4 digits of precision
     */
    public static String shorten(double r)
    {
        return String.format("%.4f", r);
    }
    
    /**
     * Constrain a value between a lower and upper limit. Good for limiting servo values etc.
     *
     * @param value The value to constrain
     * @param min   The lower limit
     * @param max   The upper limit
     * @return value if it is between min and max, otherwise if it is less than min returns min and
     * if greater than max returns max.
     */
    public static double constrain(double value, double min, double max)
    {
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
    public static double scaleRange(double x, double a, double b, double c, double d)
    {
        double o = c * (1.0 - (x - a) / (b - a)) + d * ((x - a) / (b - a));
        return o;
    }
}
