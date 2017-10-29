package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.autonomous.util.telemetry.TelemetryWrapper;

import java.util.ArrayList;
import java.util.List;

/**
 * LogBrowser - A log file browser and config file editor OpMode
 */
@TeleOp(name="Log Browser/Config Editor")
@Disabled //TODO (work in progress)
public class LogBrowser extends OpMode {

    public static class TelemetryScroller {
        private final int maxLines = 7;
        private int start = 0;
        private List<String> data;
        private boolean doUpdates = false;
        public TelemetryScroller() {
            data = new ArrayList<>();
        }

        public void startUpdating() {
            doUpdates = true;
            doUpdate();
        }

        public void stopUpdating() {
            doUpdates = false;
        }

        public void addLine(String datum) {
            data.add(datum);
            doUpdate();
        }

        public void removeLine(int index) {
            data.remove(index);
            doUpdate();
        }

        public void clear() {
            data.clear();
            doUpdate();
        }

        protected void doUpdate() {
            if (doUpdates) update();
        }

        public void update() {
            int max = Math.min(start + maxLines, data.size());
            int size = max - start;
            String[] output = new String[size];
            for (int i = start, j = 0; i < max; i++, j++) {
                output[j] = data.get(i);
            }
            TelemetryWrapper.setLines(size);
            for (int i = 0; i < size; i++) {
                TelemetryWrapper.setLine(i, output[i]);
            }
        }

        public int scrollUp(int amount) {
            if (amount < 0) return scrollDown(-amount);
            int max = Math.max(0, start - amount);
            int last = start;
            start = max;
            doUpdate();
            return start - last;
        }

        public int scrollDown(int amount) {
            if (amount < 0) return scrollUp(-amount);
            int min = Math.min(data.size() - 1, start + amount);
            int last = start;
            start = min;
            doUpdate();
            return last - start;
        }

        public void scrollTo(int position) {
            if (position > start + maxLines) {
                start = Math.min(position - maxLines, data.size() - 1);
            } else if (position < start) {
                start = Math.max(position, 0);
            }
            doUpdate();
        }
    }

    private static interface Condition {
        public boolean apply();
    }

    public static class MultiChoiceSelector extends TelemetryScroller {
        private List<String> choices = new ArrayList<>();
        private String prompt = "Choose an Option";
        private int selected = -1;
        private Gamepad gamepad;
        public MultiChoiceSelector(Gamepad gamepad) {
            this.gamepad = gamepad;
        }

        public void addChoice(String choice) {
            choices.add(choice);
        }

        public void setPrompt(String prompt) {
            this.prompt = prompt;
        }

        public int getSelection() {
            return selected;
        }

        public void setSelected(int sel) {
            selected = sel;
        }

        public void regen() {
            clear();
            addLine(prompt);
            for (int i = 0; i < choices.size(); i++) {
                if (selected == i) {
                    addLine("> " + choices.get(i));
                } else {
                    addLine("  " + choices.get(i));
                }
            }
            update();
        }

        public void removeChoice(int idx) {
            choices.remove(idx);
        }

        private void run(Condition cond) {
            regen();
            while (!cond.apply()) {
                if (gamepad.dpad_up && selected > 0) {
                    selected--;
                    regen();
                } else if (gamepad.dpad_down && selected < choices.size()-1) {
                    selected++;
                    regen();
                }
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    break;
                }
            }
            clear();
        }

    }

    private TelemetryScroller scr;
    private MultiChoiceSelector viewer;
    private boolean started = false;
    private int option = -1;

    @Override
    public void init() {
        MultiChoiceSelector selector = new MultiChoiceSelector(gamepad1);
        selector.setPrompt("Press PLAY on the phone after selecting an option:");
        selector.addChoice("Edit config");
        selector.addChoice("Read logs");
        selector.run(new Condition() {
            @Override
            public boolean apply() {
                return started;
            }
        });
        option = selector.getSelection();
    }

    public void start() {
        started = true;
    }

    @Override
    public void loop() {
        if (option == -1) {
            requestOpModeStop();
        } else if (option == 0) {
            if (viewer == null) {
                viewer = new MultiChoiceSelector(null);
                viewer.setPrompt("Edit config");

            }
        }
    }
}
