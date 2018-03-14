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
 * PositionFinder - Util for finding positions for autonomous features.
 */
@TeleOp(name="Position Collector", group="util")
public class PositionFinder extends MainTeleOp {
    private List<double[]> positions = new ArrayList<>();
    private boolean bHeld = false;
    @Override
    public void loop() {
        if (gamepad1.b) {
            if (!bHeld) {
                bHeld = true;
                positions.add(new double[]{
                       driver.getWaistPos(), driver.getShoulderPos(), driver.getElbowPos(), wrist
                        .getPosition(), getTurntablePosition(), extend.getCurrentPosition(), yaw.getPosition()
                });
            }
        } else {
            bHeld = false;
        }
        super.loop();
    }

    @Override
    public void stop() {
        if (positions.size() == 0) return; //Forget it, we don't have anything to save
        File outFile = new File(Config.storageDir + "pos_" + new SimpleDateFormat
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
