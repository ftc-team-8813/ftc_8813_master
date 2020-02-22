package org.firstinspires.ftc.teamcode.teleop;


import org.firstinspires.ftc.teamcode.common.events.Event;
import org.firstinspires.ftc.teamcode.common.events.EventBus;
import org.firstinspires.ftc.teamcode.common.events.handler.GamepadEventHandler;
import org.firstinspires.ftc.teamcode.common.util.Profiler;
import org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper;

import java.util.List;

public class MechDrive2 extends BaseTeleOp
{
    private static final int SPEED_SLOW = 0;
    private static final int SPEED_FAST = 1;
    private static final int SPEED_FASTER = 2;
    private static final int SPEED_LUDICROUS = 3;
    private static final String[] speed_modes = {"Slow", "Fast", "Faster", "Ludicrous"};
    
    
    private ButtonHelper buttonHelper;
    private int speed_mode = SPEED_FAST;
    
    private boolean intakeTrigger = false;
    private long intakeTime = 0;
    private int timeTrigger = 0;
    
    private boolean liftTrigger = false;
    
    private long start;
    
    private int updateCount;
    private long lastLog;
    
    private long lastTelemetry;
    
    private Profiler profiler;
    private static final boolean PROFILE = false;
    
    private EventBus eventBus;
    
    @Override
    public void init()
    {
        super.init();
        buttonHelper = new ButtonHelper(gamepad1);
        robot.newarm.resetArm();
    
        robot.slide.slidemotor.setPower(0.5);
        // robot.imu.stop();
        // robot.drivetrain.enableAngleCorrection();
        robot.imu.setImmediateStart(true);
        robot.imu.initialize();
        robot.drivetrain.enableAsyncLoop();
    
        profiler = new Profiler();
        
        eventBus = new EventBus();
        eventBus.addEventHandler(new GamepadEventHandler(gamepad1, gamepad2, null));
    }
    
    @Override
    public void doLoop()
    {
        long start = System.nanoTime();
        eventBus.loop();
        long elapsed = System.nanoTime() - start;
        
    }
}
