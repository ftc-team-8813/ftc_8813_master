package org.firstinspires.ftc.teamcode.common.actuators;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.sensors.Switch;
import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.firstinspires.ftc.teamcode.common.util.concurrent.GlobalThreadPool;

import java.util.Locale;
import java.util.concurrent.Future;

public class SwerveWheel
{
    private PIDMotor upper, lower;
    private Future<?> copying;
    private Future<?> calibrating;
    
    private Logger log;
    
    public SwerveWheel(PIDMotor upper, PIDMotor lower)
    {
        this.upper = upper;
        this.lower = lower;
        
        this.upper.setDirection(DcMotorSimple.Direction.FORWARD);
        this.lower.setDirection(DcMotorSimple.Direction.FORWARD);
        
        int uport = upper.getMotor().getPortNumber();
        int lport = lower.getMotor().getPortNumber();
        
        log = new Logger(String.format(Locale.US, "SwerveWheel @%d,%d", uport, lport));
    }
    
    public void setDirection(DcMotorSimple.Direction d)
    {
        log.v("Direction change -> %s", d.name());
        this.upper.setDirection(d);
        this.lower.setDirection(d);
    }
    
    public void move(int encUpper, int encLower, double power) throws InterruptedException
    {
        if (calibrating != null && !calibrating.isDone())
        {
            log.w("Calibration already in progress.");
            return;
        }
        upper.setPower(power);
        lower.setPower(power);
        
        upper.startRunToPosition(encUpper);
        lower.startRunToPosition(encLower);
        while (upper.isHolding() || lower.isHolding())
        {
            Thread.sleep(10);
        }
    }
    
    public void moveAsync(int encUpper, int encLower, double power)
    {
        if (calibrating != null && !calibrating.isDone())
        {
            log.w("Calibration already in progress.");
            return;
        }
        upper.setPower(power);
        lower.setPower(power);
    
        upper.startRunToPosition(encUpper);
        lower.startRunToPosition(encLower);
    }
    
    public void drive(double pUpper, double pLower)
    {
        upper.getMotor().setPower(pUpper);
        lower.getMotor().setPower(pLower);
    }
    
    public void stop()
    {
        upper.stopHolding();
        lower.stopHolding();
    }
    
    public int getUpperPos()
    {
        return upper.getCurrentPosition();
    }
    
    public int getLowerPos()
    {
        return lower.getCurrentPosition();
    }
    
    public void resetEncoders()
    {
        upper.stopHolding();
        lower.stopHolding();
        
        upper.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lower.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
        upper.getMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lower.getMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    public void copy(final SwerveWheel other)
    {
        log.v("Copy posision of another wheel");
        if (calibrating != null && !calibrating.isDone())
        {
            log.w("Calibration in progress.");
            return;
        }
        if (copying != null)
        {
            log.w("Already copying!");
            return;
        }
        copying = GlobalThreadPool.instance().start(() ->
        {
            upper.setPower(1);
            lower.setPower(1);
            while (true)
            {
                upper.hold(other.upper.getCurrentPosition());
                lower.hold(other.lower.getCurrentPosition());
                try
                {
                    Thread.sleep(20);
                }
                catch (InterruptedException e)
                {
                    log.d("Copier interrupted");
                    break;
                }
            }
        });
    }
    
    public void stopCopy()
    {
        log.v("Stop copying");
        if (copying == null)
        {
            return;
        }
        copying.cancel(true);
        stop();
        copying = null;
    }
    
    public void calibrate(Switch limit)
    {
        log.i("Beginning calibration sequence");
        if (calibrating != null && !calibrating.isDone())
        {
            log.w("Calibration already in progress.");
            return;
        }
        stopCopy();
        
        calibrating = GlobalThreadPool.instance().start(new Runnable()
        {
            private void seek(double power, long timeout) throws InterruptedException, IllegalStateException
            {
                long start = System.currentTimeMillis();
                while (!limit.pressed())
                {
                    SwerveWheel.this.drive(power, power);
                    if (System.currentTimeMillis() - start > timeout)
                    {
                        log.e("Timed out while seeking");
                        SwerveWheel.this.stop();
                        throw new IllegalStateException("Timed out");
                    }
                    Thread.sleep(5);
                }
                stop();
            }
            
            @Override
            public void run()
            {
                try
                {
                    seek(.8, 4000);
                    Thread.sleep(200);
                    seek(-0.3, 4000);
                    Thread.sleep(200);
                    seek(0.1, 1000);
                    resetEncoders();
                    log.i("Calibration finished.");
                } catch (InterruptedException e)
                {
                    log.v("Interrupted");
                    stop();
                }
                catch (IllegalStateException e)
                {
                    stop();
                }
            }
        });
    }
    
    public boolean calibrating()
    {
        return calibrating != null && !calibrating.isDone();
    }
}
