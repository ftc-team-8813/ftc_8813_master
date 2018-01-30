package org.firstinspires.ftc.teamcode.teleop.util;

import android.text.TextUtils;

import org.firstinspires.ftc.teamcode.teleop.MainTeleOp;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.Logger;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.zip.GZIPOutputStream;

/**
 * MoveRecorder - Saves real-time movement data of the rotating base and all servos (except the
 * claw) to a gzipped text file called move_[time].txt.gz, where [time] is the system time in
 * milliseconds when the file was created. The frames are on separate lines, and the data are
 * separated by spaces. The file starts with a header commented with '#' that contains the
 * creation time, length, and order of data.
 */

public class MoveRecorder extends MainTeleOp {
    //We'll make it floats to save RAM space
    private List<float[]> moves = new ArrayList<>();
    private int frame;
    private Logger log;

    @Override
    public void start() {
        super.start();
        frame = 0;
        log = new Logger("Move Recorder");
    }

    @Override
    public void loop() {
        super.loop();
        //Currently there is a fixed 20ms delay, giving us 50 frames per second. We will only take
        //every other frame, giving us 25fps
        if (frame % 2 == 0) {
            float[] data = {
                    (float)driver.getWaistAngle(), (float)driver.getShoulderAngle(), (float)driver.getElbowAngle(), base.getCurrentPosition(), (float)wrist.getPosition()
            };
            moves.add(data);
        }
        frame++;
    }

    @Override
    public void stop() {
        super.stop();
        try {
            //This is going to be a potentially massive text file; we will gzip it to save phone space
            PrintStream ps = new PrintStream(new GZIPOutputStream(new FileOutputStream(new File(Config.storageDir, "move_" + System.currentTimeMillis() + ".txt.gz"))));
            ps.println("# " + SimpleDateFormat.getDateTimeInstance().format(new Date()));
            ps.println("# " + moves.size() + " frames @ 25 FPS = " + (moves.size()/25.0) + " sec. Order of fields:");
            ps.println("# waist shoulder elbow base wrist");
            for (int i = 0; i < moves.size(); i++) {
                float[] m = moves.get(i);
                Float[] mm = new Float[m.length];
                for (int j = 0; j < m.length; j++) { mm[j] = m[j]; }
                ps.println(TextUtils.join(" ", mm));
            }
            ps.close();
        } catch (IOException e) {
            log.e(e);
        } finally {
            //Deallocate our large data
            moves.clear();
        }
    }
}