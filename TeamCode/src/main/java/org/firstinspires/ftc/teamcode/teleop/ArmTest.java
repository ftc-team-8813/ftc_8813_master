package org.firstinspires.ftc.teamcode.teleop;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptI2cAddressChange;
import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.util.Config;
import org.firstinspires.ftc.teamcode.util.Utils;

/**
 * ArmTest - TeleOp arm drive. Controls:
 * Left stick:
 *   Vertical: Shoulder Y +/-
 *   Horizontal: Shoulder X +/-
 * Right stick:
 *   Vertical: Elbow +/-
 *   Horizontal: N/A
 * Right Bumper: Stop
 * A: Open/Close Claw
 */
@TeleOp(name="Arm Test")
public class ArmTest extends OpMode{

    private Servo shoulderX, shoulderY, elbow, claw;
    private double turnX, turnY, turnElbow;
    private boolean claw_closed = false;
    private double maxTurn;
    private double clawCloseAmount;
    private boolean aHeld = false;
    private Config config = new Config(Config.configFile);

    @Override
    public void init() {
        shoulderX = hardwareMap.servo.get("s0"); //Assuming that s0, s1, s2, etc. are names of servos
        shoulderY = hardwareMap.servo.get("s1");
        elbow     = hardwareMap.servo.get("s2");
        claw      = hardwareMap.servo.get("s3");
        maxTurn = config.getDouble("servo_turn", 0);
        clawCloseAmount = config.getDouble("claw_closed", 0);
        claw.setPosition(0);
    }

    @Override
    public void loop() {
        //Assign values
        turnX +=  gamepad1.right_stick_x * maxTurn;
        turnY += -gamepad1.left_stick_y * maxTurn;
        turnElbow += -gamepad1.right_stick_y * maxTurn;
        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {}

        //Constrain the values
        turnX = Utils.constrain(turnX, 0, 1);
        turnY = Utils.constrain(turnY, 0, 1);
        turnElbow = Utils.constrain(turnElbow, 0, 1);

        //Set the positions
        shoulderX.setPosition(turnX);
        shoulderY.setPosition(turnY);
        elbow.setPosition(turnElbow);

        if (gamepad1.a) {
            if (!aHeld) {
                aHeld = true;
                claw_closed = !claw_closed;
                if (claw_closed)
                    claw.setPosition(clawCloseAmount);
                else
                    claw.setPosition(0);
            }
        } else {
            aHeld = false;
        }

        String fmt = "%.4f";

        telemetry.addData("X Turn", String.format(fmt, turnX));
        telemetry.addData("Y Turn", String.format(fmt, turnY));
        telemetry.addData("Elbow Turn", String.format(fmt, turnElbow));
        telemetry.addData("Claw Closed", claw_closed);
        telemetry.update();
    }
}
