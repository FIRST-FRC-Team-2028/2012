/*
 * 
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package com.phoebushighschool.phoebusrobotics.telepath;

import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.can.CANTimeoutException;

/**
 *
 * @author Administrator
 */
public class CrabDrive implements PIDOutput
{

    SteeringUnit front;
    SteeringUnit rear;
    PIDController turnController;
    GyroSensor gyro;
    public final double TURN_CONTROLLER_PROPORTIONAL = 0.06;
    public final double TURN_CONTROLLER_INTEGRAL = 0.003;
    public final double TURN_CONTROLLER_DIFFERENTIAL = 0.003;
    //public final double MIN_INPUT = -360.0;
    //public final double MAX_INPUT = 360.0;
    public final double MIN_OUTPUT = -1.0;
    public final double MAX_OUTPUT = 1.0;
    public double moveAccumulator = 0.0;
    public boolean turning = false;
    public boolean turningLeft = false;
    double kScoreThreshold = .01;
    double startTime = Timer.getFPGATimestamp();
//    UltraSonicSensor leftBallSensor;
//    UltraSonicSensor rightBallSensor;

    CrabDrive() throws CANTimeoutException
    {
        gyro = new GyroSensor(ReboundRumble.ROBOT_ANGLE_GYRO_SENSOR);
        front = new SteeringUnit(ReboundRumble.FRONT_STEERING_CAN_ID,
                ReboundRumble.FRONT_LEFT_CAN_ID,
                ReboundRumble.FRONT_RIGHT_CAN_ID);
        rear = new SteeringUnit(ReboundRumble.REAR_STEERING_CAN_ID,
                ReboundRumble.REAR_LEFT_CAN_ID,
                ReboundRumble.REAR_RIGHT_CAN_ID);
        turnController = new PIDController(TURN_CONTROLLER_PROPORTIONAL,
                TURN_CONTROLLER_INTEGRAL,
                TURN_CONTROLLER_DIFFERENTIAL,
                gyro,
                this);
        turnController.setOutputRange(MIN_OUTPUT, MAX_OUTPUT);
        turnController.disable();
//        leftBallSensor = new UltraSonicSensor(ReboundRumble.LEFT_BALL_RANGE_SENSOR);
//        rightBallSensor = new UltraSonicSensor(ReboundRumble.RIGHT_BALL_RANGE_SENSOR);
    }

    /**
     * *
     * Drive()
     *
     * this method drives the wheels
     *
     * @param outputValue a setpoint of the range -1.0 ... 0.0 ... 1.0, usually
     * a joystick
     * @throws CANTimeoutException
     * @throws NullPointerException
     */
    public void Drive(double outputValue) throws CANTimeoutException, NullPointerException
    {
        front.set(outputValue);
        rear.set(outputValue * -1.0);
    }

    /**
     * *
     * Set the output to the value calculated by PIDController. The
     * PIDController controls the rotation of the robot. PIDController outputs
     * the speed of rotation.
     *
     * @param output the value calculated by PIDController
     */
    public void pidWrite(double output)
    {
        try
        {
            if (turningLeft)
            {
                RotateLeft(-1.0 * output);
            } else
            {
                RotateRight(output);
            }
        } catch (CANTimeoutException ex)
        {
        }

    }

    /**
     * This method uses the gyro sensor to point the robot at the backboard. We
     * know we start the match facing our team's backboard and the gyro
     * initializes to 0.0 degrees (so 0.0 degrees always points towards the
     * backboard).
     *
     * @return boolean true = facing the backboard false = not yet facing the
     * backboard
     */
    public boolean FaceBackboard()
    {
        double currentAngle = gyro.getAngle();


        if (currentAngle > 0)
            currentAngle = ((((int)currentAngle) + 180) / 360) * 360.0;
        else
            currentAngle = ((((int)currentAngle) - 180) / 360) * 360.0;            
        
System.out.println("turning from " + gyro.getAngle() + " to " + currentAngle);
        return SetAngle(currentAngle);
    }

    /**
     * *
     * This method sets the absolute angle for the robot regardless of the
     * setpoint
     *
     * @param newAngle
     * @return true - it is at the setpoint false - it is not at the setpoint
     */
    public boolean SetAngle(double newAngle)
    {
        double currentAngle = gyro.getAngle();
        if (turning && newAngle > (currentAngle * 0.98) && newAngle < (currentAngle * 1.02))
        {
            DisableTurnController();
            return true;
        }
        if (!turning)
        {
            double tempCur = (currentAngle % 360) + 360;
            double tempNew = (newAngle %360) + 360;
            if ((tempCur > tempNew && tempCur <= (tempNew + 180))
                    || (tempCur <= (tempNew - 180) && tempCur > (tempNew - 360)))
                turningLeft = true;
            else
                turningLeft = false;
            System.out.println("Current Angle: " + tempCur + ", New Angle: " + tempNew + ", Turning Left" + turningLeft);
            EnableTurnController();
            turnController.setSetpoint(newAngle);
            turning = true;
        }

        return false;
    }

    /**
     * *
     * This method accumulates the angle during PID control.
     *
     * @return a more correct angle than what the gyro is returning
     */
    public double AccumulateMotion()
    {
        if (turning)
        {
            return (moveAccumulator + gyro.getAngle());
        }
        return moveAccumulator;
//        return gyro.AccumulateMotion();
    }

    /**
     * *
     * This method orients the robot until it is in line with the ball. If it is
     * already straight ahead the robot goes forward. If it is to the left of
     * the robot and seen by both sensors it rotates left. If it is to the right
     * of the robot and seen by both sensors it rotates right. If it is to the
     * left of the robot and seen only by the left sensor it rotates left. If it
     * is to the right of the robot and seen only by the right sensor it rotates
     * right.
     *
     * @throws CANTimeoutException
     */
//    public boolean GoToBall(GameMech gamemech) throws CANTimeoutException
//    {
//        double leftDistanceToBall = leftBallSensor.getDistance();
//        double rightDistanceToBall = rightBallSensor.getDistance();
//        double centerOfRobotToSensor = 21.0;
//        double centerAngle = 0.0;
//        double rotateAngle = 0.0;
//        double from90ToCenterOfRobotToSensorLine = 35.0;
//        
//        gamemech.SetLoaderOutAndUp();
//        if (!turning)
//        {
//            if (leftBallSensor.getDistance() == (rightBallSensor.getDistance() * .995)
//                    || leftBallSensor.getDistance() == (rightBallSensor.getDistance() * 1.005))
//            {
//                Drive(ReboundRumble.SEEK_BALL_SPEED);
//                System.out.println("Going forward.");
//                return true;
//            } else if (leftBallSensor.getDistance() > rightBallSensor.getDistance())
//            {
//                centerAngle = MathUtils.atan(leftDistanceToBall / centerOfRobotToSensor);
//                rotateAngle = (ConvertRadiansToDegrees(centerAngle) + from90ToCenterOfRobotToSensorLine) - 90;
//                SetAngle(rotateAngle + AccumulateMotion());
//                System.out.println("Both sensors have the ball; rotating left");
//                return true;
//            } else if (leftBallSensor.getDistance() < rightBallSensor.getDistance())
//            {
//                centerAngle = MathUtils.atan(rightDistanceToBall / centerOfRobotToSensor);
//                rotateAngle = 90 - (ConvertRadiansToDegrees(centerAngle) + from90ToCenterOfRobotToSensorLine);
//                SetAngle(rotateAngle + AccumulateMotion());
//                System.out.println("Both sensors have the ball; rotating right");
//                return true;
//            } else if (leftBallSensor.getDistance() < 60 && rightBallSensor.getDistance() > 60)//$$$ TODO fix the distance
//            {
//                RotateLeft(ReboundRumble.SEEK_BALL_SPEED);
//                System.out.println("Left sensor has the ball; rotating left.");
//                return true;
//            } else if (leftBallSensor.getDistance() > 60 && rightBallSensor.getDistance() < 60)//$$$ TODO fix this distance
//            {
//                RotateLeft(ReboundRumble.SEEK_BALL_SPEED);
//                System.out.println("Right sensor has the ball; rotating right.");
//                return true;
//            }
//        }
//        if ((leftDistanceToBall == (rightDistanceToBall * 0.995) 
//                || (leftDistanceToBall == (rightDistanceToBall * 1.005))) 
//                && (rotateAngle < 1.0 || rotateAngle > -1.0)
//                && (leftDistanceToBall < 13.0)
//                && (rightDistanceToBall < 13.0))
//        {
//            gamemech.SetLoaderOutAndDown();
//        }
//        return false;
//    }

    /**
     * *
     * This method orients the robot to the target.
     *
     * @param changeInFacing = degrees to change the robot's heading in the
     * range -90 degrees ... +90 degrees where negative values turn the robot
     * left and positive turns the robot right
     * @return true
     */
    public boolean FaceTarget(double changeInFacing)
    {
        return SetAngle(gyro.getAngle() + changeInFacing);
    }

    public double ConvertRadiansToDegrees(double radians)
    {
        return (radians * 180.0) / 3.1415926535898;
    }

    /**
     * This method enables the CrabDrive's PID controller to rotate the robot in
     * place
     *
     */
    public void EnableTurnController()
    {
        if (!turning)
        {
            turning = true;
            moveAccumulator = 0.0;
            turnController.enable();
        }
    }

    /**
     *
     * this method disables rotation of the robot with the gyro
     */
    public void DisableTurnController()
    {
        if (turning)
        {
            turnController.disable();
        }
        turning = false;
    }

    /**
     * *
     * enable()
     *
     * this method enables control for the steering and drive and resets the
     * gyro. it is only called when driver station is enabled.
     *
     * @throws CANTimeoutException
     * @throws NullPointerException
     */
    public void enable() throws CANTimeoutException, NullPointerException
    {
        rear.enableTurnControl();
        rear.enableSpeedControl();
        front.enableTurnControl();
        front.enableSpeedControl();
        gyro.reset();
    }

    /**
     * *
     * disable()
     *
     * this method disables control for the steering and drive
     *
     * @throws CANTimeoutException
     * @throws NullPointerException
     */
    public void disable() throws CANTimeoutException, NullPointerException
    {
        rear.disableTurnControl();
        rear.disableSpeedControl();
        front.disableTurnControl();
        front.disableSpeedControl();
    }

    /**
     * *
     * CrabSteer()
     *
     * this method turns the wheels to the same angle allowing the robot to
     * 'crab' across the floor
     *
     * @param outputValue is the position to steer the wheels to in the range of
     * -1.0 ... 0.0 ... 1.0 where 0.0 is centered, 1.0 is ______ and -1.0 is
     * ______
     */
    public void CrabSteer(double outputValue) throws CANTimeoutException
    {
        front.Steer(outputValue);
        rear.Steer(outputValue);
    }

    /**
     * *
     * SlewSteer()
     *
     * this method turns the front and rear wheels to opposite angles from each
     * other allowing the robot to turn in a circle
     *
     * @param outputValue is the position to steer the wheels to in the range of
     * -1.0 ... 0.0 ... 1.0 where 0.0 is centered, 1.0 is ______ and -1.0 is
     * ______
     */
    public void SlewSteer(double outputValue) throws CANTimeoutException
    {
        front.Steer(outputValue * 0.75);
        rear.Steer((outputValue * 0.75) * -1.0);

    }

    /**
     * *
     * RotateLeft()
     *
     * Turns the wheels 90 degrees and allows the whole robot to rotate about
     * it's center axis at a speed specified by the parameter 'speed'.
     *
     * @param speed The speed to power the wheels while rotating in the range of
     * -1.0 ... 0.0 ... 1.0
     *
     * @return boolean true - indicates the drive is being controlled by this
     * method, joystick control should not be allowed
     */
    public boolean RotateRight(double speed) throws CANTimeoutException
    {
        front.RotateLeft(speed);
        rear.RotateLeft(speed);
        return true;
    }

    /**
     * *
     * RotateRight()
     *
     * Turns the wheels 90 degrees and allows the whole robot to rotate about
     * it's center axis at a speed specified by the parameter 'speed'.
     *
     * @param speed The speed to power the wheels while rotating in the range of
     * -1.0 ... 0.0 ... 1.0
     *
     * @return boolean true - indicates the drive is being controlled by this
     * method, joystick control should not be allowed
     */
    public boolean RotateLeft(double speed) throws CANTimeoutException
    {
        front.RotateRight(speed);
        rear.RotateRight(speed);
        return true;
    }

    /**
     * *
     * This method rotates the robot 90 degrees in preparation for going on the
     * ramp.
     *
     * @return true
     */
    public boolean RotateRobot90Degrees()
    {
        if (!turning)
        {
            if ((AccumulateMotion() > 0 && AccumulateMotion() <= 180)
                    || (AccumulateMotion() <= -180 && AccumulateMotion() > -360))
            {
                turningLeft = true;
            } else
            {
                turningLeft = false;
            }
            turnController.enable();
            turnController.setSetpoint(90.0);
            turning = true;
        }
        return true;
    }

    /**
     * *
     * MoveToLeft90()
     *
     * this method moves the wheels to left 90 degrees
     *
     * @throws CANTimeoutException
     */
    public void MoveToLeft90() throws CANTimeoutException
    {
        front.MoveToLeft90();
        rear.MoveToLeft90();
    }

    /**
     * *
     * MoveToRight90()
     *
     * this method moves the wheels to right 90 degrees
     *
     * @throws CANTimeoutException
     */
    public void MoveToRight90() throws CANTimeoutException
    {
        front.MoveToRight90();
        rear.MoveToRight90();
    }

    public double getPosition() throws CANTimeoutException
    {
        return front.getPosition();
    }

    public String toString()
    {
        return "Rear:" + rear + ", Front:" + front;
    }
}