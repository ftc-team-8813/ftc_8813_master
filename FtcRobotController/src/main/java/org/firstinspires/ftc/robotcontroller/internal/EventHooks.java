package org.firstinspires.ftc.robotcontroller.internal;

/**
 * EventHooks - Event hooks for starting/stopping the OpenCV camera viewer from reading frames when
 * the app is not active.
 */

public interface EventHooks {
    public void stop();
    public void resume();
}
