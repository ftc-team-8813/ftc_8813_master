package org.firstinspires.ftc.teamcode.teleop;

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
                       driver.getWaistPos(), driver.getShoulderPos(), driver.getElbowPos()
                });
            }
        } else {
            bHeld = false;
        }
        super.loop();
    }

    @Override
    public void stop() {
        File outFile = new File(Config.storageDir + "pos_" + new SimpleDateFormat("yyMMdd_HHmmss").format(new Date()) + ".txt");
        try (FileWriter w = new FileWriter(outFile)) {
            for (int i = 0; i < positions.size(); i++) {
                double[] pos = positions.get(i);
                w.write(i + ":\n\tw: " + pos[0] + "\n\ts: " + pos[1] + "\n\te: " + pos[2] + "\n");
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
        if (outFile.exists()) {
            Utils.scanFile(outFile);
        }
    }
}
