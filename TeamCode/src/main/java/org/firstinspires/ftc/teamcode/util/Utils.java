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
 * Created by aidan on 10/13/17.
 */

public class Utils {
    public static double constrain(double value, double min, double max) {
        return (value < min) ? min : ((value > max) ? max : value);
    }

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
