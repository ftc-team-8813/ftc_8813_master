package org.firstinspires.ftc.teamcode.teleop.util;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * ArmDriver - Drive the arm servos based on controller input.
 */

public class ArmDriver {
    public static final double SERVO_MAX_ANGLE = 7.04614352305139340626;
    /**
     * ArmPos - Describes the position of the arm
     */
    private static class ArmPos {
        private final double w, s, e;
        public ArmPos(double waist, double shoulder, double elbow) {
            this.w = waist;
            this.s = shoulder;
            this.e = elbow;
        }

        public double getWaistRadians() {
            return w;
        }

        public double getShoulderRadians() {
            return s;
        }

        public double getElbowRadians() {
            return e;
        }

    }


    private final Servo ws, ss, es;
    private double[] currentPositions = new double[3];

    public ArmDriver(Servo waist, Servo shoulder, Servo elbow) {
        ws = waist;
        ss = shoulder;
        es = elbow;
    }

    public void setWaistAngle(double radians) {
        currentPositions[0] = getServoPosition(radians);
    }



    //TODO: Convert to servo coordinates (0..1) and back

    private static double getServoPosition(double radians) {
        return radians/SERVO_MAX_ANGLE;
    }

    private static double getAngle(double servoPosition) {
        return servoPosition*SERVO_MAX_ANGLE;
    }
}
