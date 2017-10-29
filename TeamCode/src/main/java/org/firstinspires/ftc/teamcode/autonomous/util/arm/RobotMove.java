package org.firstinspires.ftc.teamcode.autonomous.util.arm;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

/**
 * RobotMove - a simple class that holds all the motor positions of the arm in a single instance of
 * time.
 */

public class RobotMove {
    /** X (base servo) rotation */
    public double xTurn;
    /** Y (base up/down servo) rotation */
    public double yTurn;
    /** Elbow servo rotation */
    public double elbow;
    /** Current power of base motor */
    public double basePower;
    /** Current power of extend motor */
    public double extendPower;
    /** Claw servo rotation */
    public double clawPos;
    /** Number of milliseconds since last command */
    public int dt;
    /** Whether or not the limit switch has been pressed */
    public boolean extSet;

    /**
     * Write the data to a file.
     * @param out The stream to write to
     * @throws IOException If an I/O error occurs.
     */
    public void write(DataOutputStream out) throws IOException {
        out.writeInt(dt);
        //Servos only have a precision of ~0.004 so we don't need >20 digits of precision
        out.writeFloat((float)xTurn);
        out.writeFloat((float)yTurn);
        out.writeFloat((float)elbow);
        out.writeFloat((float)clawPos);

        out.writeFloat((float)basePower);
        out.writeFloat((float)extendPower);

        out.writeByte(extSet ? 0xFF : 0);
    }

    /**
     * Read the data from a file.
     * @param in The stream to read from
     * @throws IOException If an I/O error occurs.
     */
    public void read(DataInputStream in) throws IOException {
        dt = in.readInt();

        xTurn = in.readFloat();
        yTurn = in.readFloat();
        elbow = in.readFloat();
        clawPos = in.readFloat();

        basePower = in.readFloat();
        extendPower = in.readFloat();

        extSet = in.readByte() != 0;
    }

    @Override
    public boolean equals(Object other) {
        if (other == null) return false;
        if (!(other instanceof RobotMove)) return false;
        RobotMove r = (RobotMove)other;
        return xTurn == r.xTurn && yTurn == r.yTurn && elbow == r.elbow && clawPos == r.clawPos
                && basePower == r.basePower && extendPower == r.extendPower;
    }
}
