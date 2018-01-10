package org.firstinspires.ftc.teamcode.autonomous.util.arm;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.util.Config;

/**
 * Arm - Utility class to control all of the servos in the arm. Helpful for holding all of the
 * servos in one class so they can be passed between different methods easily.
 */

public class Arm {
    private Servo waist, shoulder, elbow, claw, wrist;
    private boolean claw_closed = false;
    private Config conf;

    public Arm(Config conf, Servo waist, Servo shoulder, Servo elbow, Servo claw, Servo wrist) {
        this.conf = conf;
        this.waist = waist;
        this.shoulder = shoulder;
        this.elbow = elbow;
        this.claw = claw;
        this.wrist = wrist;
    }

    public Arm(Servo waist, Servo shoulder, Servo elbow, Servo claw, Servo wrist) {
        this(BaseAutonomous.instance().config, waist, shoulder, elbow, claw, wrist);
    }

    public void moveTo(double w, double s, double e) {
        waist.setPosition(w);
        shoulder.setPosition(s);
        elbow.setPosition(e);
    }

    public void moveWrist(double wr) {wrist.setPosition(wr);}

    public void moveClaw(double c) {
        claw.setPosition(c);
    }

    public void moveWaist(double w) {
        waist.setPosition(w);
    }

    public void moveShoulder(double s) {
        shoulder.setPosition(s);
    }

    public void moveElbow(double e) {
        elbow.setPosition(e);
    }

    public void openClaw() {
        claw_closed = false;
        claw.setPosition(conf.getDouble("claw_open", 0));
    }

    public void closeClaw() {
        claw_closed = true;
        claw.setPosition(conf.getDouble("claw_closed", 0));
    }

    public void toggleClaw() {
        if (claw_closed) {
            openClaw();
        } else {
            closeClaw();
        }
    }

    public boolean isClawClosed() {
        return claw_closed;
    }

    public void setClaw(boolean closed) {
        if (closed) {
            closeClaw();
        } else {
            openClaw();
        }
    }

    public Servo getWaist() {
        return waist;
    }

    public Servo getShoulder() {
        return shoulder;
    }

    public Servo getElbow() {
        return elbow;
    }
}
