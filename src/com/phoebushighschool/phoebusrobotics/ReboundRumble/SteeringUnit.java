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
public class SteeringUnit {

    public static final double kProportional = 500.0;
    public static final double kIntegral = 0.35;
    public static final double kDifferential = 0.005;
//    public static final double NEUTRAL_POSITION_SETPOINT = 0.5;
    public static final double ROTATE_SETPOINT_LEFT_90 = -1.0;
    public static final double ROTATE_SETPOINT_RIGHT_90 = 1.0;
    double speed2;
    Wheel left;
    Wheel right;
    SteeringMotor middle;
    CrabDrive drive;
    double setpoint;
    FrcMath math = new FrcMath();


    SteeringUnit(int steeringCANID, int leftCanID, int rightCanID) throws CANTimeoutException {
        middle = new SteeringMotor(steeringCANID, kProportional, kIntegral, kDifferential);
        left = new Wheel(leftCanID);
        right = new Wheel(rightCanID);
        setpoint = 0.0;
    }

    /**
     * *
     * set()
     *
     * this method sets the output value for the wheels
     *
     * @param outputValue
     * @throws CANTimeoutException
     */
    public void set(double outputValue) throws CANTimeoutException {
        left.set(outputValue);
        right.set(outputValue);
    }
    /*
     * AdjustTurnSpeed()
     *
     * This method will prevent tipping of the robot by compensating turn speed
     * for drive speed. The faster the operator is driving, the slower the turn
     * speed will be.
     *
     * @param joystickValue , speed
     */

    public double AdjustTurnSpeed(double joystickValue, double speed) {
        speed2 = (speed - 1.0) / (-333.33);
        return speed2;
    }

    /**
     * *
     * enableSpeedControl()
     *
     * this method enables speed control for the wheels
     *
     * @throws CANTimeoutException
     */
    public void enableSpeedControl() throws CANTimeoutException {
        left.enableSpeedControl();
        right.enableSpeedControl();
    }

    /**
     * *
     * disableSpeedControl()
     *
     * this method enables speed control for the wheels
     *
     * @throws CANTimeoutException
     */
    public void disableSpeedControl() throws CANTimeoutException {
        left.disableSpeedControl();
        right.disableSpeedControl();
    }

    /**
     * *
     * enableTurnControl()
     *
     * this method enables steering for that motor
     *
     * @throws CANTimeoutException
     * @throws NullPointerException
     */
    public void enableTurnControl() throws CANTimeoutException, NullPointerException {
        middle.enableControl();
    }

    /**
     * *
     * disableSpeedControl()
     *
     * this method disables steering for that motor
     *
     * @throws CANTimeoutException
     * @throws NullPointerException
     */
    public void disableTurnControl() throws CANTimeoutException, NullPointerException {
        middle.disableControl();
    }

    /**
     * *
     * ConvertJoystickToPosition()
     *
     * this method converts a joystick value(-1.0 ... 0.0 ... 1.0) to a position
     * value 0.0 ... 1.0
     *
     * @param outputValue a setpoint of the range -1.0 ... 0.0 ... 1.0
     * @return position a setpoint of the range 0.0 ... 1.0
     */
    public static double ConvertJoystickToPosition(double outputValue) {
        double position = ((outputValue * 0.26) + 0.55);
        return position;
    }

    /**
     * *
     * Steer()
     *
     * Steer sets the position setpoint of the closed-loop control system for
     * the robot's steering.
     *
     * @param outputValue a setpoint in the range of -1.0 ... 0 ... 1.0 where
     * -1.0 is full counter clockwise and 1.0 is full clockwise
     * @return boolean true - the setpoint was changed false - the setpoint was
     * not changed
     */
    public boolean Steer(double outputValue) throws CANTimeoutException {

        //System.out.println("SteeringUnit.Steer(" + outputValue + "): setpoint " + setpoint);
        if (setpoint == outputValue) {
            return false;
        }
        setpoint = outputValue;
        // convert joystick postition to a sensor reading
        outputValue = ConvertJoystickToPosition(outputValue);
        middle.setX(outputValue);
        return true;
    }

    /**
     * *
     * Turns the wheels to 90 degrees and then turn the wheels at what the speed
     * is set at
     *
     * @param speed -1.0 ... 0.0 ... 1.0
     */
    public void RotateLeft(double speed) throws CANTimeoutException {
        if (!Steer(ROTATE_SETPOINT_LEFT_90)) {
            double error = middle.getPosition()
                    - ConvertJoystickToPosition(ROTATE_SETPOINT_LEFT_90);
            //System.out.println(error);
            if (error < TwentyTwelve.JOYSTICK_DEADBAND
                    && error > (-1.0 * TwentyTwelve.JOYSTICK_DEADBAND)) {
                left.set(speed);
                right.set(speed);
            }
        }
    }

    /**
     * *
     * RotateRight()
     *
     * Turns the wheels to 90 degrees and then turn the wheels at what the speed
     * is set at
     *
     * @param speed -1.0 ... 0.0 ... 1.0
     */
    public void RotateRight(double speed) throws CANTimeoutException {
        if (!Steer(ROTATE_SETPOINT_RIGHT_90)) {
            double error = middle.getPosition()
                    - ConvertJoystickToPosition(ROTATE_SETPOINT_RIGHT_90);
            //System.out.println(error);
            if (error < TwentyTwelve.JOYSTICK_DEADBAND
                    && error > (-1.0 * TwentyTwelve.JOYSTICK_DEADBAND)) {
                left.set(speed);
                right.set(speed);
            }
        }
    }

    /**
     * *
     * MoveToLeft90()
     *
     * Turns the wheels to the left 90 degrees
     *
     * @throws CANTimeoutException
     */
    public void MoveToLeft90() throws CANTimeoutException {
        Steer(ROTATE_SETPOINT_LEFT_90);
    }

    /**
     * *
     * MoveToRight90()
     *
     * Turns the wheels to the right 90 degrees
     *
     * @throws CANTimeoutException
     */
    public void MoveToRight90() throws CANTimeoutException {
        Steer(ROTATE_SETPOINT_RIGHT_90);
    }

    public String toString() {
        String message = new String();
        message = "Pos: ";
        try {
            message += math.Normalize(middle.getPosition()) + ", Set: "
                    + math.Normalize(ConvertJoystickToPosition(setpoint));
        } catch (CANTimeoutException e) {
            message = "Steering posistion: AAAARRRGGGHHH!!!";
        }
//        message += "Left: " + left + ", Right: " + right;
        return message;
    }

    /**
     *
     * @return @throws CANTimeoutException
     */
    public double getPosition() throws CANTimeoutException {
        return middle.getPosition();
    }
}
