/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package com.phoebushighschool.phoebusrobotics.ReboundRumble;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import java.util.Timer;
import java.util.TimerTask;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * @author jmiller015 This is the code for the slingshot portion of the 2012
 * game Rebound Rumble
 */
public class SlingShot
{

    public final double LOAD_POSITION = 0.94;
    public final double SHOOT_POSITION = 0.86;
    public static final double SHOOTER_PULL_SPEED = -1.0;   // $$$ ToDo:  determine the maximum speed for the shooter pull motor
    public static final double SHOOTER_ARM_SPEED = 1.0;     // $$$ ToDo:  determine the maximum speed to safely re-arm the shooter
    public static final double ELEVATION_PID_PROPORTIONAL = 550.0;
    public static final double ELEVATION_PID_INTEGRAL = 0.225;
    public static final double ELEVATION_PID_DERIVATIVE = 0.003;
    public static final double ELEVATION_ERROR = 0.05;
    CANJaguar shooterPull;
    CANJaguar elevation;
    Relay trigger;
//    DigitalInput loadPosition;
//    Relay slingMagnet;
    boolean ballLoaded = false;
    boolean settingForce = false;
    boolean timerDone = false;
    boolean isElevationPIDControlled = false;
    boolean isPulledBack = false;
    // public DigitalInput ballSensor;
    Timer timer;
    SlingShotTimer timerTask;
    double elevationSetpoint;

    // $$$ todo; determine if shooter has ultrasonic range sensor
    public class SlingShotTimer extends TimerTask
    {

        SlingShot slingShot;

        public SlingShotTimer(SlingShot pull)
        {
            slingShot = pull;
        }

        /**
         * This method is automatically invoked when the timer task expires
         *
         */
        public void run()
        {
            slingShot.ForceIsSet();           // Tell SlingShot the force is now set
        }
    }

    public SlingShot()
    {
        try
        {
            shooterPull = new CANJaguar(TwentyTwelve.SHOOTER_PULL_CAN_ID);
            shooterPull.changeControlMode(CANJaguar.ControlMode.kPercentVbus);
            shooterPull.configNeutralMode(CANJaguar.NeutralMode.kBrake);
            elevation = new CANJaguar(TwentyTwelve.ELEVATION_CAN_ID);
            SetElevationPositionControl();
        } catch (CANTimeoutException ex)
        {
        }
        trigger = new Relay(TwentyTwelve.SHOOTER_TRIGGER_RELAY_CHANNEL);
//        loadPosition = new DigitalInput(ReboundRumble.LOAD_POSITON_LIMIT_SWITCH);
//        slingMagnet = new Relay(ReboundRumble.SLING_ELECTROMAGNET_RELAY_CHANNEL);
        //  ballSensor = new DigitalInput(ReboundRumble.SHOOTER_BALL_SENSOR_GPIO_CHANNEL);
        settingForce = false;
    }

    /**
     * Puts the elevation Jaguar in position control mode using the angle sensor
     * to set the position.
     *
     * @throws CANTimeoutException
     */
    protected void SetElevationPositionControl() throws CANTimeoutException
    {
        elevation.changeControlMode(CANJaguar.ControlMode.kPosition);
        elevation.setPositionReference(CANJaguar.PositionReference.kPotentiometer);
        elevation.setPID(ELEVATION_PID_PROPORTIONAL, ELEVATION_PID_INTEGRAL, ELEVATION_PID_DERIVATIVE);
        elevation.configNeutralMode(CANJaguar.NeutralMode.kBrake);
        isElevationPIDControlled = true;
    }

    /**
     * Puts the elevation Jaguar in percent voltage mode so the joystick can be
     * used to set the elevation
     */
    protected void SetElevationPercentVbusControl() throws CANTimeoutException
    {
        elevation.changeControlMode(CANJaguar.ControlMode.kPercentVbus);
        elevation.configNeutralMode(CANJaguar.NeutralMode.kBrake);
        isElevationPIDControlled = false;
    }

    /**
     * AdjustShooter()
     *
     * This method will move the quick release along the slingshot guides.
     *
     *
     * @param outputValue this takes a setpoint for the motor controller in the
     * range -1.0 ... 0.0 ... 1.0 where 1.0 is applying maximum force, and -1.0
     * returns to the rearm position.
     */
    public void AdjustShooter(double outputValue) throws CANTimeoutException
    {
        if (!settingForce)
        {
            shooterPull.setX(outputValue);
        }
    }

    /**
     * SetShooterForce()
     *
     * This method will set the shooter force as specified by Aim() given a
     * percentage input (0.0% ... 100.0%).
     *
     * @param percentForce - percentage of shooter force in the range 0.0% ...
     * 100.0%
     *
     * @return true - the force is set false - the force is not yet set.
     *
     * @throws CANTimeoutException
     */
    public boolean SetShooterForce(double percentForce, double adjust) throws CANTimeoutException
    {
        double time = percentForce;          // in seconds
//        if (percentForce < 0.1)
//        {
//            return true;
//        }
        if (timerDone)
        {
            return true;
        }
        time = 4.0 + adjust + (0.04 * percentForce);
        if (!settingForce)
        {
            if (Arm())
            {
                settingForce = true;
                shooterPull.setX(SHOOTER_PULL_SPEED);
                timer = new Timer();
                timerTask = new SlingShotTimer(this);
                timer.schedule(timerTask, (long) time * 1000);
            }
        }
        if (shooterPull.getReverseLimitOK() == false)
        {
            shooterPull.setX(0.0);
            trigger.set(Relay.Value.kOff);
            return true;
        }
        return false;
    }

    /**
     * isShooterAtShootPositiion()
     *
     * This method will set the shooter to the shoot position
     *
     * @return
     * @throws CANTimeoutException
     */
    public boolean isShooterAtShootPosition() throws CANTimeoutException
    {
        //SetElevation(SHOOT_POSITION);
        double error = elevation.getPosition() - SHOOT_POSITION;
        if (error <= ELEVATION_ERROR && error >= -1.0 * ELEVATION_ERROR)
        {
            return true;
        }
        return false;
    }

    /**
     * *
     * isShooterAtLoadPosition()
     *
     * This method returns if the shooter is at the load position.
     *
     * @return true - shooter is at load position false - shooter is not at load
     * position
     *
     * @throws CANTimeoutException
     */
    public boolean isShooterAtLoadPosition() throws CANTimeoutException
    {
        //SetElevation(LOAD_POSITION);
        double error = elevation.getPosition() - LOAD_POSITION;
        System.out.println("elevation position: " + elevation.getPosition());
        if (error <= ELEVATION_ERROR && error >= -1.0 * ELEVATION_ERROR)
        {
            return true;
        }
        return false;
    }

    /**
     * SetElevation()
     *
     * This method will set the shooter to the angle specified by Aim()
     *
     * @param setPoint - angle reading in the range 0.1 ... 1.0
     *
     * @return boolean - true = the elevation is at the setpoint false = the
     * elevation is not yet at the setpoint
     */
    public boolean SetElevation(double setPoint) throws CANTimeoutException
    {
        elevationSetpoint = setPoint;
        if (!isElevationPIDControlled)
        {
            SetElevationPositionControl();
        }
        elevation.setX(setPoint);
        double angle = elevation.getPosition();
        if ((elevationSetpoint - angle) <= 0.05 && (elevationSetpoint - angle) >= -0.05)
        {
            return true;
        }
        return false;
    }

    public boolean PrimaryPullBack() throws CANTimeoutException
    {
        double time = 0.5;
        if (Arm())
        {
            settingForce = true;
            shooterPull.setX(SHOOTER_PULL_SPEED);
            timer = new Timer();
            timerTask = new SlingShotTimer(this);
            timer.schedule(timerTask, (long) time * 1000);
        }
        if (timerDone)
        {
            return true;
        }
        return false;
    }

    /**
     * Controls the output voltage to the elevation motor using the Jaguar in
     * percent Vbus mode.
     *
     * @param outputValue
     * @return
     * @throws CANTimeoutException
     */
    public boolean AdjustElevation(double outputValue) throws CANTimeoutException
    {
        if (isElevationPIDControlled)
        {
            SetElevationPercentVbusControl();
        }
        elevation.setX(outputValue);
//        System.out.println(elevation.getPosition());
//        double delta = SHOOT_POSITION - LOAD_POSITION;
//        outputValue = ((outputValue + 1.0) / 2.0) * 0.1 ;
//        elevation.setX(SHOOT_POSITION + outputValue);

        return true;
    }

    /**
     * Arm()
     *
     * This method will make the trigger on, and locked.
     *
     * @return boolean - true = the sling shot is armed and ready to go false =
     * the sling shot is still moving to the arm position
     */
    public boolean Arm() throws CANTimeoutException
    {
        if (shooterPull.getForwardLimitOK())
        {
            shooterPull.setX(SHOOTER_ARM_SPEED);
            return false;
        }
        if (shooterPull.getForwardLimitOK() == false)
        {
            shooterPull.setX(0.0);
            trigger.set(Relay.Value.kForward);
//            slingMagnet.set(Relay.Value.kReverse);                  // $$$ ToDo: verify direction from wiring
            timerDone = false;
            isPulledBack = false;
            return true;
        }
        return false;
    }

    /**
     * release()
     *
     * This method will turn off the trigger, thus releasing the ball.
     */
    public void Release()
    {
        trigger.set(Relay.Value.kOff);
//        slingMagnet.set(Relay.Value.kOff);
        ballLoaded = false;
        settingForce = false;
        timerDone = false;
        timer = null;
    }

    /**
     *
     * SetLoadPosition()
     *
     * tells the elevation motor to move the shooter to the elevation setpoint.
     *
     * @return true if the loader is at the load elevation false if the loader
     * is not yet at the load elevation
     *
     * @throws CANTimeoutException
     */
    public boolean SetLoadPosition() throws CANTimeoutException
    {
//        if (!loadPosition.get())
//        {
//            return true;
//        }
        AdjustElevation(1.0);
////        double error = elevation.getPosition() - LOAD_POSITION;
////        if (error <= ELEVATION_ERROR && error>= -1.0 * ELEVATION_ERROR)
//        if (!loadPosition.get())
//        {
//            return true;
//        }
        if (!elevation.getForwardLimitOK())
            return true;
        return false;
    }

    public boolean SetShootPosition() throws CANTimeoutException
    {
        AdjustElevation(-1.0);
//        double error = elevation.getPosition() - SHOOT_POSITION;
//        if (error <= ELEVATION_ERROR && error>= -1.0 * ELEVATION_ERROR)
        if (!elevation.getReverseLimitOK())
        {
            return true;
        }
        return false;
    }

    /**
     * *
     * Enable()
     *
     * This method enables elevation control
     *
     * @throws CANTimeoutException
     */
    public void Enable() throws CANTimeoutException
    {
        elevation.enableControl();
    }

    /**
     * Disable()
     *
     * This method disables elevation control
     *
     * @throws CANTimeoutException
     */
    public void Disable() throws CANTimeoutException
    {
        elevation.disableControl();
    }

    public boolean IsPulledBack()
    {
        return isPulledBack;
    }

    /**
     * ForceIsSet()
     *
     * This method sets a flag indicating the force setpoint has been reached.
     * It is called by the SlingShotTimer when the timer has triggered. Once
     * called the force can be adjusted.
     *
     */
    public void ForceIsSet()
    {
        try
        {
            shooterPull.setX(0.0);          // Turn off the force motor
        } catch (CANTimeoutException ex)
        {
        }
        settingForce = false;           // Tell SlingShot we are no longer setting the force
        timerDone = true;               // Tell SlingShot the timer is done
        isPulledBack = true;
    }
}