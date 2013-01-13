/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package com.phoebushighschool.phoebusrobotics.telepath;

import edu.wpi.first.wpilibj.AnalogModule;

/**
 *
 * @author David
 */
public class UltraSonicSensor 
{
    AnalogModule ultrasonicSensor;
    int channel;
    
    /**
     * Constructor that uses the default module slot
     * 
     * @param analogChannel channel the ultrasonic sensor is connected to on the
     *                      cRIO 9201 module.
     */
    public UltraSonicSensor(int analogChannel)
    {
        channel = analogChannel;
        ultrasonicSensor =  AnalogModule.getInstance(1);
    }
    
    
    /**
     * Return the actual distance to a target in inches.
     *
     * The distance is based on the current voltage values.
     * 
     * @returns double - distance to the target in inches
     */
    public double getDistance()
    {
        double value = ultrasonicSensor.getVoltage(channel);
        return (value / (5.0 / 512.0));
    }
    
    public double pidGet()
    {
        return getDistance();
    }
    
    public double getVoltage()
    {
        return ultrasonicSensor.getVoltage(channel);
    }
}
