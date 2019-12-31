package org.firstinspires.ftc.teamcode.autonomous;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.common.util.DataStorage;
import org.firstinspires.ftc.teamcode.common.util.Logger;

import java.io.File;
import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.autonomous.util.opmodes.AutoDesigner.OP_ARM;
import static org.firstinspires.ftc.teamcode.autonomous.util.opmodes.AutoDesigner.OP_CLAW;
import static org.firstinspires.ftc.teamcode.autonomous.util.opmodes.AutoDesigner.OP_FORWARD;
import static org.firstinspires.ftc.teamcode.autonomous.util.opmodes.AutoDesigner.OP_HOOK;
import static org.firstinspires.ftc.teamcode.autonomous.util.opmodes.AutoDesigner.OP_INTAKE;
import static org.firstinspires.ftc.teamcode.autonomous.util.opmodes.AutoDesigner.OP_LIFT;
import static org.firstinspires.ftc.teamcode.autonomous.util.opmodes.AutoDesigner.OP_RIGHT;
import static org.firstinspires.ftc.teamcode.autonomous.util.opmodes.AutoDesigner.OP_TURN;

@Autonomous(name="Autonomous Player")
public class AutonomousPlayer extends BaseAutonomous
{
    
    private int op;
    private int[] params;
    private Robot robot;
    private DataStorage data;
    private Logger log;
    
    @Override
    public void run() throws InterruptedException
    {
        log = new Logger("Autonomous Playback");
        
        robot = Robot.instance();
        
        data = new DataStorage(new File(Config.storageDir + "autonomous.json"));
        
        int op_count = data.getInt("op_count", 0);
        for (int i = 0; i < op_count; i++)
        {
            fetchOp(i);
            log.d("Running op: %d %s", op, Arrays.toString(params));
            executeOp(1);
            Thread.sleep(500);
        }
    }
    
    private void fetchOp(int n)
    {
        JsonObject operation = (JsonObject)data.getJson(Integer.toString(n));
        op = operation.get("op").getAsInt();
        JsonArray paramsJson = operation.getAsJsonArray("params");
        params = new int[paramsJson.size()];
        for (int i = 0; i < params.length; i++)
        {
            params[i] = paramsJson.get(i).getAsInt();
        }
    }
    
    // TODO Unify this with the AutoDesigner
    private void executeOp(int direction) throws InterruptedException
    {
        if (op == OP_FORWARD)
        {
            robot.drivetrain.move(0.3, 0, 0, params[0] * direction);
        }
        else if (op == OP_RIGHT)
        {
            robot.drivetrain.move(0, 0.3, 0, params[0] * direction);
        }
        else if (op == OP_TURN)
        {
            robot.drivetrain.move(0, 0, 0.3, params[0] * direction);
        }
        else if (op == OP_HOOK)
        {
            boolean closed = (params[0] == 1);
            if (direction == -1) closed = !closed;
            if (closed) robot.foundationhook.moveHookDown();
            else robot.foundationhook.moveHookUp();
        }
        else if (op == OP_INTAKE)
        {
            robot.intake.collectStone(params[1]/100.0 * direction);
            Thread.sleep(params[0]);
            robot.intake.stopIntake();
        }
        else if (op == OP_LIFT)
        {
            int pos = params[0];
            if (direction == -1) pos = params[1];
            robot.slide.raiseLiftEnc(pos);
        }
        else if (op == OP_ARM)
        {
            int pos = params[0];
            if (direction == -1) pos = params[1];
            robot.newarm.moveArmTo(0.4, pos);
        }
        else if (op == OP_CLAW)
        {
            boolean closed = (params[0] == 1);
            if (direction == -1) closed = !closed;
            if (closed) robot.claw.closeClaw();
            else robot.claw.openClaw();
        }
    }
}
