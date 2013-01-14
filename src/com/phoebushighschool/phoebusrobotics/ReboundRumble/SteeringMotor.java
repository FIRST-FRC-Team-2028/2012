/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package com.phoebushighschool.phoebusrobotics.ReboundRumble;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.can.CANTimeoutException;

/**
 *
 * @author Justin
 */
public class SteeringMotor {

    CANJaguar motor;
    SteeringUnit steer;
    public double motorVoltage = TwentyTwelve.MAX_MOTOR_VOLTAGE;

    SteeringMotor(int steeringCANID, double kProportional, double kIntegral, double kDifferential) throws CANTimeoutException {
        motor = new CANJaguar(steeringCANID, CANJaguar.ControlMode.kPosition);
        motor.configMaxOutputVoltage(motorVoltage);
        motor.configPotentiometerTurns(1);
        motor.setPositionReference(CANJaguar.PositionReference.kPotentiometer);
        motor.configNeutralMode(CANJaguar.NeutralMode.kBrake);
        motor.setPID(kProportional, kIntegral, kDifferential);
    }

    public void enableControl() throws CANTimeoutException {
        motor.enableControl();
    }

    public void disableControl() throws CANTimeoutException {
        motor.disableControl();
    }

    public void setX(double outputValue) throws CANTimeoutException {
        motor.setX(outputValue);
    }

    public double getPosition() throws CANTimeoutException {
        return motor.getPosition();
    }
}
