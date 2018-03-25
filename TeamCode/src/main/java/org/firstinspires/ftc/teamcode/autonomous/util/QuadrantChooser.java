package org.firstinspires.ftc.teamcode.autonomous.util;

import android.annotation.SuppressLint;
import android.graphics.BitmapFactory;
import android.view.MotionEvent;
import android.view.View;
import android.widget.ImageView;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.Persistent;
import org.firstinspires.ftc.teamcode.util.Utils;

/**
 * QuadrantChooser -- Choose a quadrant of the field by tapping an image displayed on the RC.
 */

public class QuadrantChooser {

    private Telemetry telemetry;
    private int quadrant;

    public QuadrantChooser(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * Choose a quadrant. Blocks until one is available; if the thread is interrupted, will
     * return -1.
     * @return a quadrant, or -1 if the thread is interrupted
     */
    public int chooseQuadrant() {
        telemetry.addData("","Please choose a quadrant on the robot controller");
        telemetry.update();
        final FtcRobotControllerActivity activity = (FtcRobotControllerActivity) AppUtil.getInstance()
                .getActivity();
        final int center_x = 210;
        final int center_y = 244;
        final int w = 421, h = 559;
        final Logger log = new Logger("Quadrant Chooser");
        activity.runOnUiThread(new Runnable() {
            @SuppressLint("ClickableViewAccessibility")
            @Override
            public void run() {
                ImageView image = new ImageView(activity);
                image.setImageBitmap(BitmapFactory.decodeResource(activity.getResources(), R
                        .drawable.field));
                activity.cameraMonitorLayout.addView(image);
                image.setOnTouchListener(new View.OnTouchListener() {
                    @Override
                    public boolean onTouch(View v, MotionEvent event) {
                        if (event.getAction() == MotionEvent.ACTION_DOWN) {
                            int touch_x = (int)event.getX();
                            int touch_y = (int)event.getY();
                            log.d("Received touch event at (%d, %d)", touch_x, touch_y);
                            int dx = (int)-v.getX();
                            int dy = (int)-v.getY();
                            int vw = (int)v.getWidth();
                            int vh = (int)v.getHeight();
                            int px = (touch_x + dx) * w / vw;
                            int py = (touch_y + dy) * h / vh;
                            log.d("Click event at (%d,%d); transformed to image pixel (%d, %d)",
                                    touch_x, touch_y, px, py);
                            if (px < center_x) {
                                if (py < center_y) {
                                    quadrant = 1;
                                } else {
                                    quadrant = 2;
                                }
                            } else {
                                if (py < center_y) {
                                    quadrant = 4;
                                } else {
                                    quadrant = 3;
                                }
                            }
                            activity.cameraMonitorLayout.removeView(v);
                            return true;
                        }
                        return false;
                    }
                });
                log.i("Listening for click events");
            }

            private void setQuadrant(int q) {
                quadrant = q;
            }
        });
        quadrant = 0;
        while (quadrant == 0) {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                return -1;
            }
        }
        log.i("Quadrant chosen: " + Utils.quadName(quadrant));
        telemetry.clear();
        telemetry.addData("Quadrant chosen", Utils.quadName(quadrant));
        telemetry.update();
        Persistent.put("quadrant", quadrant);
        return quadrant;
    }
}
