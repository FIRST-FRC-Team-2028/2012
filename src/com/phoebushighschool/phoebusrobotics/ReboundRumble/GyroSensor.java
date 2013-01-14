/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package com.phoebushighschool.phoebusrobotics.ReboundRumble;
import edu.wpi.first.wpilibj.Gyro;

/**
 *
 * @author David
 */
public class GyroSensor extends Gyro
{
    public GyroSensor(int channel)
    {
        super(channel);
    }
    
    /***
     * this method is required by the Gyro class
     * @return 
     */
    public double pidGet() 
    {
        return super.getAngle();
    }
    
    /***
     * This method gets the angle of the gyro and therefore the robot.
     * @return 
     */
    public double getAngle()
    {
        return pidGet();
    }
    
    /***
     * This method returns the angle of the robot in the range of -360 ... 0.0 ... 360.
     * @return 
     */
    public double readAngle()
    {
        double temp = getAngle();
        temp = temp % 360;
        return temp;
    }
}
