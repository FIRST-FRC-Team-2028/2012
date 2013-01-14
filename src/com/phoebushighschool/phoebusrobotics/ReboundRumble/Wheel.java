/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package com.phoebushighschool.phoebusrobotics.ReboundRumble;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.can.CANTimeoutException;

/**
 *
 * @author Administrator
 */
public class Wheel {

    public static final double kProportional = 0.5;
    public static final double kIntegral = 0.005;
    public static final double kDifferential = 0.015;
    public static final double kVoltageDivider = 2.5;
    public static final double kVoltageRampRate = 12.0;

    double setpoint = 0.0;
    CANJaguar motorController;
    FrcMath math = new FrcMath();

    Wheel(int canId) throws CANTimeoutException {
        motorController = new CANJaguar(canId);       
        motorController.configMaxOutputVoltage(TwentyTwelve.MAX_MOTOR_VOLTAGE);
        motorController.changeControlMode(CANJaguar.ControlMode.kVoltage);
        motorController.configNeutralMode(CANJaguar.NeutralMode.kBrake);
    }

    /***
     * set()
     * 
     * this method sets the output value for one of the wheels.  It converts the
     * joystick input (-1.0... 0.0 ... 1.0) into RPM.
     * 
     * @param outputValue a setpoint of the range -1.0 ... 0.0 ...1.0
     * @throws CANTimeoutException 
     */
    public void set(double outputValue) throws CANTimeoutException
    {
        outputValue = (outputValue / 1.45 * TwentyTwelve.MAX_MOTOR_VOLTAGE);
        if (setpoint != outputValue)
        {
            setpoint = outputValue;
            motorController.setX(setpoint);
        }
    }
    
    /***
     * enableSpeedControl()
     * 
     * this method enables speed control for the wheels
     * @throws CANTimeoutException 
     */
    public void enableSpeedControl() throws CANTimeoutException
    {
        motorController.enableControl();
    }
    
    /***
     * disableSpeedControl()
     * 
     * this method disables speed control for the wheels
     * 
     * @throws CANTimeoutException 
     */
    public void disableSpeedControl() throws CANTimeoutException
    {
        motorController.disableControl();
    }
    
    public String toString()
    {
        String message;
        try
        {
            message = "{speed: " + math.Normalize(motorController.getSpeed()) +
                    ", setpoint: " + math.Normalize(setpoint) + "}";
        } catch (CANTimeoutException ex)
        {
            message = "Wheel speed: AAAARRRGGGHHH!!!";
        }
        return message;
    }
}
