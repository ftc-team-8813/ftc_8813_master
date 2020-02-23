package org.firstinspires.ftc.teamcode.common.sensors;

import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.firstinspires.ftc.teamcode.common.util.concurrent.GlobalThreadPool;

public class Odometry
{
    private OdometryEncoder fwdEnc;
    private OdometryEncoder strafeEnc;
    private IMU imu;
    
    private volatile double prevEncX, prevEncY;
    private volatile double x, y;
    
    private Logger log;
    
    public Odometry(OdometryEncoder forward, OdometryEncoder strafe, IMU imu)
    {
        this.fwdEnc = forward;
        this.strafeEnc = strafe;
        this.imu = imu;
        log = new Logger("Odometry");
        GlobalThreadPool.instance().start(() ->
        {
           while (true)
           {
               update();
               try
               {
                   Thread.sleep(10);
               }
               catch (InterruptedException e)
               {
                   break;
               }
           }
        });
    }
    
    /*
           | +Y __,
           |,--)
    -------+------> Forward
           |     +X
           v
           Strafe
     */
    
    private synchronized void update()
    {
        double encX = fwdEnc.getPosition();
        double encY = -strafeEnc.getPosition();
        double heading = Math.toRadians(imu.getHeading());
        
        double deltaX = encX - prevEncX;
        double deltaY = encY - prevEncY;
        prevEncX = encX;
        prevEncY = encY;
        
        double realDx = deltaX * Math.cos(heading) - deltaY * Math.sin(heading);
        double realDy = deltaY * Math.cos(heading) + deltaX * Math.sin(heading);
        
        x += realDx;
        y += realDy;
        /*
        if (realDx != 0 || realDy != 0)
        {
            log.d("Delta: measured <%.3f,%.3f> -> <%.3f,%.3f>", deltaX, deltaY, realDx, realDy);
        }
         */
    }
    
    public synchronized double getForwardDistance()
    {
        return x;
    }
    
    public synchronized double getStrafeDistance()
    {
        return -y;
    }
    
    public synchronized void reset()
    {
        x = 0;
        y = 0;
    }
}
