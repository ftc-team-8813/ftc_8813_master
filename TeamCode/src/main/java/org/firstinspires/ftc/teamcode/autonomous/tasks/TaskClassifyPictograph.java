package org.firstinspires.ftc.teamcode.autonomous.tasks;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.util.opencv.CameraStream;
import org.firstinspires.ftc.teamcode.autonomous.util.opencv.PictographFinder;
import org.firstinspires.ftc.teamcode.util.Logger;

import java.util.ArrayList;
import java.util.List;

/**
 * TaskClassifyPictograph - Classify a pictograph in the camera viewport. Should be instantiated in
 * initialize() and then executed with runTasks() before obtaining the classification result. Blocks
 * until classification result is obtained, but tasks can be added by {@link #runTask()} to run in
 * parallel.
 */

public class TaskClassifyPictograph implements Task {

    private CameraStream stream;
    private List<Task> taskList = new ArrayList<>();
    private Logger log = new Logger("Pictograph Finder");

    public enum Result {
        NONE, LEFT, RIGHT, CENTER
    }

    private Result prevResult;

    public Result getResult() {
        return prevResult;
    }

    public TaskClassifyPictograph() {
        log.v("Initializing camera stream");
        stream = BaseAutonomous.instance().getCameraStream();
    }


    public void addTask(Task t) {
        taskList.add(t);
    }

    @Override
    public void runTask() throws InterruptedException {
        PictographFinder finder = new PictographFinder();
        stream.addListener(finder);
        long end = -1;
        while (!taskList.isEmpty()) {
            taskList.remove(0).runTask();
        }
        while (!finder.finished() && !Thread.interrupted()) {
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                return;
            }
        }
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
