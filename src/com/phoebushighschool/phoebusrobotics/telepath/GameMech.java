/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package com.phoebushighschool.phoebusrobotics.telepath;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.image.NIVisionException;

/**
 *
 * @author jmiller015
 */
public class GameMech
{

    Loader load;
    SlingShot shoot;
    CameraUnit camera;
    Compressor airCompressor;
//    RampGoDownerrrr PushDownerrr;
    boolean usingLoader;

    public GameMech()
    {
        load = new Loader();
        shoot = new SlingShot();
        camera = new CameraUnit();
//        PushDownerrr = new RampGoDownerrrr();
        airCompressor = new Compressor(ReboundRumble.AIR_PRESSURE_SENSOR_GPIO_CHANNEL,
                ReboundRumble.AIR_COMPRESSOR_RELAY_CHANNEL);

    }

    /**
     * ReloadShooter()
     *
     * Move the sling shot elevation to the load position. Once the shooter is
     * in the load position, run the loader's conveyor.
     *
     * @return boolean - true if conveyor is running false if shooter is not in
     * load position or loader is not retracted
     */
    public boolean ReloadShooter() throws CANTimeoutException
    {
        //shoot.SetElevation(shoot.LOAD_POSITION);
        boolean isRetracted = load.LoaderRetract();
        boolean isInLoadPosition = true;
        // isInLoadPosition = shoot.isShooterAtLoadPosition();
//        if (isRetracted && isInLoadPosition)
        {
            //shoot.PullBackHalfSecond();
            load.EnableConveyor(Loader.CONVEYOR_RELOAD_SPEED);
            return true;
        }
        //return false;
    }

    /**
     * Shoot()
     *
     * This method shoots by turning off the electromagnet. Do I need to say
     * more?
     *
     * @return
     */
    public boolean Shoot(CrabDrive drive, double forceAdjust) throws CANTimeoutException
    {
//        if (Aim(drive, forceAdjust))          // $$$ ToDo:  Uncomment
//        {
        shoot.Release();
        Timer.delay(0.05);
        shoot.Arm();
        shoot.SetLoadPosition();
//        }
        return true;
    }

    /**
     * LoadBall()
     *
     * This method will deploy the loader, set the load position, and load the
     * ball
     *
     * @return true - if a ball has been loaded false - if a ball has not yet
     * been loaded
     */
//    public boolean LoadBall() throws CANTimeoutException
//    {
//        boolean isLoaderDeployed = load.LoaderDeploy();
//        boolean isShooterInLoadPosition = shoot.SetLoadPosition();
//        boolean isArmed = shoot.Arm();
//        if (isLoaderDeployed && isShooterInLoadPosition)
//        {
//            load.EnableConveyor(1.0);
//            if (shoot.ballLoaded == false)
//            {
//                load.EnableConveyor(1.0);
//                return false;
//            } else if (shoot.ballLoaded)
//            {
//                load.DisableConveyor();
//                return true;
//            }
//        }
//        return false;
//    }
    /**
     * IsBallLoaded()
     *
     * Is a ball loaded?
     *
     * @return
     */
    public boolean IsBallLoaded()
    {
        return true;
    }

    public void StartConveyor(double speed)
    {
        load.EnableConveyor(speed);
    }

    public void StopConveyor()
    {
        load.DisableConveyor();
    }

    /**
     *
     * @return
     */
//    public void DeployRampGoDownerrrr()
//    {
//        ProcessLoader();
//        PushDownerrr.PushRampGoDownerrrrOut();
//    }

//    public void RetractRampGoDownerrrr()
//    {
//        PushDownerrr.PullRampGoDownerrrrIn();
//    }

    /**
     * *
     * This method moves the camera to the load ball position.
     */
    public void ProcessLoader()
    {
        if (usingLoader)
        {
            camera.SetLoadPosition();
        }
    }

    /**
     * Enable()
     *
     * this method enables the GameMech
     *
     */
    public void Enable() throws CANTimeoutException
    {
        airCompressor.start();
        shoot.Enable();
        camera.SetNeutralPosition();
    }

    /**
     * Disable()
     *
     * this method disables the GameMech
     */
    public void Disable() throws CANTimeoutException
    {
        airCompressor.stop();
        shoot.Disable();
    }
//
//    /**
//     * DeployLoader()
//     *
//     * this method will deploy the loader
//     *
//     * @return true if the loader is deployed false if the loader is not yet
//     * deployed
//     */
    public boolean DeployLoader() throws CANTimeoutException
    {
        usingLoader = true;
        ProcessLoader();
        shoot.SetLoadPosition();
        //if (shoot.isShooterAtLoadPosition())
            return load.LoaderDeploy();
        //else 
        //    return false;
    }
//
//    /**
//     * RetractLoader()
//     *
//     * This method will retract the loader
//     *
//     * @return true if the loader is retracted false if the loader is not yet
//     * retracted
//     */
    public boolean RetractLoader()
    {
        if (usingLoader)
        {
            camera.SetNeutralPosition();
        }
        load.LoaderRetract();
        usingLoader = false;
        return load.LoaderRetract();
    }

    /**
     * LaunchBallWithCamera
     *
     * this method shoots after aiming
     *
     * @throws NIVisionException
     */
    public void LaunchBallWithCamera(CrabDrive drive, double forceAdjust) throws CANTimeoutException
    {
        if (Aim(drive, forceAdjust))
        {
            shoot.Release();
        }
    }

    public boolean SetShooterForce(double percent, double adjust) throws CANTimeoutException
    {
        return shoot.SetShooterForce(percent, adjust);
    }

    /**
     * Aim()
     *
     * This method will aim the entire robot at the leftmost target in the
     * camera's field of view. It will set the SlingShot's elevation, the
     * SlingShot's force and rotate the robot to point at the target
     *
     * @return true - the robot is ready to shoot and score false - the robot is
     * not yet completely aimed at the target
     */
    public boolean Aim(CrabDrive drive, double forceAdjust) throws CANTimeoutException
    {

        if (camera != null && camera.r != null)
        {
            if (camera.FindTarget())
            {
                boolean isElevationSet;
                boolean isForceSet;
                boolean isAngleSet;
//                isForceSet = shoot.SetShooterForce(1.0, forceAdjust);
                isForceSet = shoot.SetShooterForce(camera.GetShooterForce(camera.GetDistance()), forceAdjust);
//                if (load.isLoaderIn())
//                {
                isElevationSet = shoot.SetShootPosition();
//                }
                isAngleSet = drive.FaceTarget(camera.cameraPan.getAngle() - 85);
                if (isElevationSet && isForceSet && isAngleSet)
                {
                    return true;
                }
            }
        }
        return false;
    }

    public boolean ReArmShooter() throws CANTimeoutException
    {
        return shoot.Arm();
    }

    /**
     * AdjustShooterForce()
     *
     * This method will set the shooter force as according to the shooter stick
     * right joystick between -1.0 ... 0.0 ... 1.0
     *
     * @param outputValue
     * @throws CANTimeoutException
     */
    public void AdjustShooterForce(double outputValue) throws CANTimeoutException
    {
        shoot.AdjustShooter(outputValue);
    }

    public void AdjustShooterElevation(double outputValue) throws CANTimeoutException
    {
        shoot.AdjustElevation(outputValue);
    }

    public void SetLoadElevation() throws CANTimeoutException
    {
        shoot.SetElevation(shoot.LOAD_POSITION);
    }

    public void/*boolean*/ SetShootElevation() throws CANTimeoutException
    {
        /*return*/shoot.SetElevation(shoot.SHOOT_POSITION);
    }

    /**
     * MoveToCameraAngle()
     *
     * This centers the targets on the camera
     */
    public void MoveToCameraAngle() throws CANTimeoutException
    {
        camera.getAngleToTarget();
        if (camera.getAngleToTarget() >= -43)
        {
            System.out.println("Target Angle is" + camera.getAngleToTarget());
        } else if (camera.getAngleToTarget() <= 43)
        {
            System.out.println("Target Angle is" + camera.getAngleToTarget());
        }
    }
//
//    /**
//     *
//     *
//     * @return
//     */
    public boolean SetLoaderOutAndUp() throws CANTimeoutException
    {
        boolean isLoaderUp = load.LoaderUp();
        boolean isLoaderOut = load.LoaderOut();
        ProcessLoader();
        shoot.SetLoadPosition();
        if (isLoaderUp && isLoaderOut)
        {
            return true;
        }
        return false;
    }
//
//    /**
//     *
//     *
//     *
//     * @return
//     */
    public boolean SetLoaderOutAndDown()
    {
        if (load.LoaderOut())
        {
            if (load.LoaderDown())
            {
                return true;
            }
        }
        return false;
    }
//
//    /**
//     *
//     *
//     *
//     * @return
//     */
    public boolean SetLoaderInAndUp()
    {
        if (load.LoaderUp())
        {
            if (load.LoaderIn())
            {
                return true;
            }
        }
        return false;
    }
//        /**
//     * LoadBall()
//     *
//     * This method will deploy the loader, set the load position, and pickup a
//     * ball from the floor into the load
//     *
//     * @return true - if a ball has been loaded false - if a ball has not yet
//     * been loaded
//     */
//    public boolean LoadBallIntoLoader() throws CANTimeoutException
//    {
//        //boolean isShooterInLoadPosition = shoot.SetLoadPosition();
//        //if (!isShooterInLoadPosition)
//        //    return false;
//        boolean isLoaderOut = load.isLoaderOut();
//        if (isLoaderOut/* && isShooterInLoadPosition*/)
//        {
//            load.EnableConveyor(Loader.CONVEYOR_PICKUP_SPEED);
////            if (load.ballSensor.get())
////            {
////                load.DisableConveyor();
////                return true;
////            }
//        }
//        return false;
//    }
    public boolean isLoaderIn()
    {
        return load.isLoaderIn();
    }
    public boolean isLoaderOut()
    {
        return load.isLoaderOut();
    }
    public boolean isLoaderUp()
    {
        return load.isLoaderUp();
    }
    public boolean isLoaderDown()
    {
        return load.isLoaderDown();
    }
}