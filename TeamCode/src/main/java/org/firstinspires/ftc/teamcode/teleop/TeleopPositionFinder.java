package org.firstinspires.ftc.teamcode.teleop;

import android.text.TextUtils;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.Utils;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/**
 * TODO Position finder for TeleOp. Stores arm position as waist, adj, dist, wrist, yaw, base, ext
 */
// Extend PositionFinder because we want MainTeleOp to detect it as that and allow us to use B.
// Otherwise, we will override all of the functions.
@TeleOp(name="TeleOp Position Finder", group="util")
public class TeleopPositionFinder extends PositionFinder {
    private List<double[]> positions = new ArrayList<>();
    private boolean bHeld = false;
    @Override
    public void run() {
        if (gamepad1.b) {
            if (!bHeld) {
                bHeld = true;
                positions.add(new double[]{
                        controller.i(), controller.j(), controller.k(), controller.w(), controller.claw(), controller.yaw(), controller.base(), controller.extend()
                });
            }
        } else {
            bHeld = false;
        }
    }

    @Override
    public void stop() {
        if (positions.size() == 0) return; //Forget it, we don't have anything to save
        File outFile = new File(Config.storageDir + "pos_teleop_" + new SimpleDateFormat
                ("yyMMdd_HHmmss", Locale.US).format(new Date()) + ".txt");
        try (FileWriter w = new FileWriter(outFile)) {
            int i = 0;
            for (double[] pos : positions) {
                //Iteratively convert to Double[] because we need an array of objects, not
                // primitives, so that we can use TextUtils.join
                Double[] vals = new Double[pos.length];
                for (int j = 0; j < vals.length; j++) {
                    vals[j] = pos[j];
                }
                w.write(i + ": " + TextUtils.join(", ", vals) + "\n");
                i++;
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
        if (outFile.exists()) {
            Utils.scanFile(outFile);
        }
        super.stop();
    }
}
