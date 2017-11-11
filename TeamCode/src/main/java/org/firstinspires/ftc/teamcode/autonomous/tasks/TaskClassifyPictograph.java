package org.firstinspires.ftc.teamcode.autonomous.tasks;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.util.opencv.CameraStream;
import org.firstinspires.ftc.teamcode.autonomous.util.opencv.PictographFinder;

/**
 * TaskClassifyPictograph - Classify a pictograph in the camera viewport. Should be instantiated in
 * initialize() and then executed with runTasks() before obtaining the classification result. Blocks
 * until classification result is obtained.
 */

public class TaskClassifyPictograph implements Task {

    private CameraStream stream;

    public enum Result {
        NONE, LEFT, RIGHT, CENTER
    }

    private Result prevResult;

    public Result getResult() {
        return prevResult;
    }

    public TaskClassifyPictograph() {
        stream = BaseAutonomous.instance().getCameraStream();
    }

    @Override
    public void runTask() throws InterruptedException {
        PictographFinder finder = new PictographFinder();
        stream.addListener(finder);
        while (!finder.finished() && !Thread.interrupted());
        stream.removeListener(finder);
        if (Thread.interrupted()) return;
        //Otherwise, we may have gotten a result!
        PictographFinder.ClassificationType classification = finder.getPrevClassification();

        switch (classification.name) {
            case "Left":
                prevResult = Result.LEFT;
                break;
            case "Right":
                prevResult = Result.RIGHT;
                break;
            case "Center":
                prevResult = Result.CENTER;
                break;
            default:
                prevResult = Result.NONE;
                break;
        }

    }
}
