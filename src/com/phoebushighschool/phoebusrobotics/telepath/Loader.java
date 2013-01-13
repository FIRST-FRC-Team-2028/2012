/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package com.phoebushighschool.phoebusrobotics.telepath;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Victor;

/**
 *
 * @author jmiller015 This is the code for the 2012 loader portion for the game
 * Rebound Rumble
 */
public class Loader
{

    Victor Conveyor;
    Solenoid loaderInSolenoid;
    Solenoid loaderOutSolenoid;
    Solenoid loaderUpSolenoid;
    Solenoid loaderDownSolenoid;
    DigitalInput loaderIn;
    DigitalInput loaderOut;
    DigitalInput loaderUp;
    DigitalInput loaderDown;
    DigitalInput ballSensor;
    public final static double CONVEYOR_RELOAD_SPEED = -1.0;
    public final static double CONVEYOR_PICKUP_SPEED = -0.5;
    public final static double CONVEYOR_DOWN_SPEED = 0.33;
    public boolean isBallLoaded = false;

    public Loader()
    {
        Conveyor = new Victor(ReboundRumble.LOADER_CONVEYOR_PWM);
        loaderInSolenoid = new Solenoid(ReboundRumble.LOADER_IN_SOLENOID_CHANNEL);
        loaderOutSolenoid = new Solenoid(ReboundRumble.LOADER_OUT_SOLENOID_CHANNEL);
        loaderUpSolenoid = new Solenoid(ReboundRumble.LOADER_UP_SOLENOID_CHANNEL);
        loaderDownSolenoid = new Solenoid(ReboundRumble.LOADER_DOWN_SOLENOID_CHANNEL);
        loaderIn = new DigitalInput(ReboundRumble.LOAD_IN_GPIO_CHANNEL);
        loaderOut = new DigitalInput(ReboundRumble.LOAD_OUT_GPIO_CHANNEL);
        loaderUp = new DigitalInput(ReboundRumble.LOAD_UP_GPIO_CHANNEL);
        loaderDown = new DigitalInput(ReboundRumble.LOAD_DOWN_GPIO_CHANNEL);
        loaderDownSolenoid.set(false);
        loaderUpSolenoid.set(true);
        loaderOutSolenoid.set(false);
        loaderInSolenoid.set(true);
    }

    /**
     * EnableConveyor()
     *
     * this method will turn on the conveyor.
     */
    public void EnableConveyor(double speed)
    {
        Conveyor.set(speed);
    }

    /**
     * DisableConveyor()
     *
     * this method will turn off the conveyor.
     */
    public void DisableConveyor()
    {
        Conveyor.set(0.0);
    }

    /**
     * LoadBall()
     *
     * This method will enable the conveyor until a ball is detected by the ball
     * sensor.
     *
     * @return true when a ball has been detected at the ball sensor. false if
     * the ball is not yet at the sensor and the conveyor is still running
     */
//    public boolean LoadBall()
//    {
//        if (LoaderDeploy())
//        {
//            if (true)
//            {
//                EnableConveyor(0.25);
//                return false;
//            }
//            DisableConveyor();
//            isBallLoaded = true;
//            return true;
//        }
//        return false;
//    }
//
//    public boolean getBallSensor()
//    {
//        return ballSensor.get();
//    }

    /**
     * IsBallLoaded()
     *
     * This method tells you if a ball is loaded in the loader
     *
     * @return
     */
    public boolean IsBallLoaded()
    {
        return isBallLoaded;
    }
//
//    /**
//     * LoaderDeploy()
//     *
//     * this method will put the loader out and down.
//     *
//     * @return true when the loader is deployed fully false when the loader is
//     * not yet fully deployed
//     */
    public boolean LoaderDeploy()
    {
        loaderInSolenoid.set(false);
        loaderOutSolenoid.set(true);
        if (isLoaderOut())
        {
            loaderUpSolenoid.set(false);
            loaderDownSolenoid.set(true);

            if (isLoaderDown())
            {
                return true;
            }
        }
        return false;
    }
//
//    /**
//     * LoaderRetract()
//     *
//     * This method will bring the loader in and up
//     *
//     * @return true when the loader is retracted false when the loader is not
//     * yet fully retracted
//     */
    public boolean LoaderRetract()
    {
        DisableConveyor();

        loaderDownSolenoid.set(false);
        loaderUpSolenoid.set(true);
        if (isLoaderUp())
        {
            loaderOutSolenoid.set(false);
            loaderInSolenoid.set(true);
            if (isLoaderIn())
            {
                return true;
            }
        }
        return false;
    }
//
//    /**
//     * isLoaderOut()
//     *
//     * This method tells you if the loader is out or not
//     *
//     * @return boolean - true - the loader is all the way out. ` false - the
//     * loader is not all the way out.
//     */
    public boolean isLoaderOut()
    {
        if (loaderOut.get() == false)
        {
            return true;
        }
        return false;
    }
//
//    /**
//     * isloaderDown()
//     *
//     * This method will tell you if the loader is all the way down
//     *
//     * @return boolean - true - the loader is all the way down. false - the
//     * loader is not all the way down.
//     */
    public boolean isLoaderDown()
    {
        if (loaderDown.get() == false)
        {
            return true;
        }
        return false;
    }
//
//    /**
//     * isLoaderUp()
//     *
//     * This method will tell you if the loader is up or not.
//     *
//     * @return boolean - true - the loader is all the way up false - the loader
//     * is not all the way up
//     */
    public boolean isLoaderUp()
    {
        if (loaderUp.get() == false)
        {
            return true;
        }
        return false;
    }
//
//    /**
//     * isLoaderIn()
//     *
//     * This method tells you if the loader is all the way in or not
//     *
//     * @return boolean - true - the loader is all the way in. false - the loader
//     * is not all the way in.
//     */
    public boolean isLoaderIn()
    {
        if (loaderIn.get() == false)
        {
            return true;
        }
        return false;
    }
//    /**
//     * LoaderOut()
//     * 
//     * This method puts the loader in the fully out position.
//     * 
//     * @return boolean - true - the loader is all the way out
//     *                   false - the loader is not yet all the way out
//     */
    public boolean LoaderOut()
    {
        loaderOutSolenoid.set(true);
        loaderInSolenoid.set(false);
        if (loaderOut.get() == false)
        {
            return true;
        }
        return false;
    }
//    /**
//     * LoaderIn()
//     * 
//     * This method puts the loader in the fully In position.
//     * 
//     * @return boolean - true - the loader is all the way in
//     *                   false - the loader is not yet all the way in
//     */
    public boolean LoaderIn()
    {
        loaderInSolenoid.set(true);
        loaderOutSolenoid.set(false);
        if (loaderIn.get() == false)
        {
            return true;
        }
        return false;
    }
//    /**
//     * LoaderUp()
//     * 
//     * This method puts the loader in the fully up position.
//     * 
//     * @return boolean - true - the loader is all the way up
//     *                   false - the loader is not yet all the way up
//     */
    public boolean LoaderUp()
    {
        loaderUpSolenoid.set(true);
        loaderDownSolenoid.set(false);
        if (loaderUp.get() == false)
        {
            return true;
        }
        return false;
    }
//    /**
//     * LoaderDown()
//     * 
//     * This method puts the loader in the fully down position.
//     * 
//     * @return boolean - true - the loader is all the way down
//     *                   false - the loader is not yet all the way down
//     */
    public boolean LoaderDown()
    {
        loaderDownSolenoid.set(true);
        loaderUpSolenoid.set(false);
        if (loaderDown.get() == false)
        {
            return true;
        }
       return false; 
    }
}