/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package com.phoebushighschool.phoebusrobotics.ReboundRumble;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.can.CANTimeoutException;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class TwentyTwelve extends SimpleRobot {
    // These are the CAN ID identifications 

    public static final int SHOOTER_PULL_CAN_ID = 26 /*
             * 2
             */;    //$$$ TODO: Check this value.
    public static final int ELEVATION_CAN_ID = 27 /*
             * 5
             */;       //$$$ TODO: Chech this value.
    public static final int FRONT_LEFT_CAN_ID = 28;
    public static final int FRONT_RIGHT_CAN_ID = 22;
    public static final int REAR_LEFT_CAN_ID = 1;
    public static final int REAR_RIGHT_CAN_ID = 29;
    public static final int FRONT_STEERING_CAN_ID = 14;
    public static final int REAR_STEERING_CAN_ID = 24;
    // These are the Solenoid Channel assignments
    public static final int LOADER_UP_SOLENOID_CHANNEL = 5;
    public static final int LOADER_DOWN_SOLENOID_CHANNEL = 6;
    public static final int LOADER_IN_SOLENOID_CHANNEL = 3;
    public static final int LOADER_OUT_SOLENOID_CHANNEL = 4;
    // These are the digital input channel assignments
    public static final int LOAD_IN_GPIO_CHANNEL = 5;               // input
    public static final int LOAD_OUT_GPIO_CHANNEL = 4;              // input
    public static final int LOAD_UP_GPIO_CHANNEL = 2;               // input
    public static final int LOAD_DOWN_GPIO_CHANNEL = 3;             // input
    public static final int ANGLE_ELEVATION_GPIO_CHANNEL = 6;       // power only
    public static final int ANGLE_FRONT_STEER_GPIO_CHANNEL = 7;     // power only
    public static final int ANGLE_REAR_STEER_GPIO_CHANNEL = 8;      // power only
    public static final int KEY_SENSOR_GPIO_CHANNEL = 9;            // input
    public static final int SHOOTER_BALL_SENSOR_GPIO_CHANNEL = 11;  // input
    public static final int LOADER_BALL_SENSOR_GPIO_CHANNEL = 12;   // input
    public static final int AIR_PRESSURE_SENSOR_GPIO_CHANNEL = 14;  // input
    // These are the digital sidecar relay assignments
    public static final int SHOOTER_TRIGGER_RELAY_CHANNEL = 3;      // output
    public static final int AIR_COMPRESSOR_RELAY_CHANNEL = 4;       // output
    public static final int SLING_ELECTROMAGNET_RELAY_CHANNEL = 5;  // output
    // These are the PWM assignments
    public static final int LOADER_CONVEYOR_PWM = 1;
//    public final static int LOADER_BALL_BRUSH_PWM = 2;
//    public static final int LOAD_POSITON_LIMIT_SWITCH = 3;
    public static final int CAMERA_AZIMUTH_PWM = 6;
    public static final int CAMERA_ELEVATION_PWM = 7;
    // These are the Analog channel assignments
    public static final int ROBOT_ANGLE_GYRO_SENSOR = 1;
    public static final int BACKBOARD_RANGE_SENSOR = 2;
    public static final int LEFT_BALL_RANGE_SENSOR = 3;
    public static final int RIGHT_BALL_RANGE_SENSOR = 4;
    // These are the finetuners for the force, elevation, and convayor speed
    public static final int ANALOG_FORCE_INPUT = 1;
    public static final int ANALOG_ELEVATION_INPUT = 2;
    public static final int QUADRATURE_ENCODER_COUNTS_PER_REV = 360;
    public static final double PERIOD = 0.10;
    public static final double MAX_MOTOR_VOLTAGE = 12;
    public static final String CAMERA_IP = "10.20.28.11";
    public static final double JOYSTICK_DEADBAND = 0.05;
    public static final double SEEK_BALL_SPEED = 0.25;
    public static final boolean DEBUG = true;
    public static final double G_ACCEL = 9.81;
    GamePadF310 driveStick = new GamePadF310(1);
    GamePadF310 shooterStick = new GamePadF310(2);
    CrabDrive drive;
    GameMech game;
    //DigitalInput gyro = new DigitalInput(ROBOT_ANGLE_GYRO_SENSOR);
    DriverStation driverI;

    public TwentyTwelve() {
        super();
        if (DEBUG) {
            System.out.println("Entering Rebound Rumble constructor.");
        }
        try {
            drive = new CrabDrive();
        } catch (CANTimeoutException e) {
            System.out.println(e);
        }
        driverI = DriverStation.getInstance();
        game = new GameMech();
        if (DEBUG) {
            System.out.println("Exiting Rebound Rumble constructor.");
        }

    }

    /**
     * This function is called once each time the robot enters autonomous mode.
     */
    public void autonomous() {
//        double sForceA = (driverI.getAnalogIn(ANALOG_FORCE_INPUT) / 3.3) - 0.5;
//        double sElevationA = (driverI.getAnalogIn(ANALOG_ELEVATION_INPUT) / 3.3) - 0.5;
//        RobotState state = new RobotState();
//        try
//        {
//            if (drive != null)
//            {
//                drive.enable();
//            }
//
//            if (game != null)
//            {
//                game.Enable();
//            }
        while (isEnabled() && isAutonomous()) {
//                System.out.println(state);
////                if (game.SetShootElevation())
////                {
//                    switch (state.GetState())
//                    {
//                        case RobotState.Aim:
//                            if (game.SetShooterForce(100.0, sForceA))
//                            {
//                                state.NextState();
//                            }
//                            break;
//                        case RobotState.Shoot:
//                            if (game.Shoot(drive, sForceA))
//                            {
//                                state.NextState();
//                            }
//                            break;
//                        case RobotState.Reload:
//                            game.AdjustShooterElevation(1.0);
//                            game.ReArmShooter();
//                            if (false /*
//                                     * game.ReloadShooter()
//                                     */)
//                            {
//                                state.NextState();
//                            }
//                            break;
//                    }
//                }
            Timer.delay(PERIOD);
            getWatchdog().feed();
        }
//            if (drive != null)
//            {
//                drive.disable();
//            }
//            if (game != null)
//            {
//                game.Disable();
//            }
//        } catch (CANTimeoutException ex)
//        {
//            System.out.println(ex);
//        }
    }

    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {
        try {
            if (driveStick == null) {
                throw new NullPointerException("Cannot create DualAction joystick");
            }
            if (drive == null) {
                throw new NullPointerException("Cannot create CrabDrive");
            }
            if (drive != null) {
                drive.enable();
            }

            if (game != null) {
                game.Enable();
            }
            int count = 0;

            boolean ballLoaded = false;
            while (isEnabled() && isOperatorControl()) {
                double sForceA = (driverI.getAnalogIn(ANALOG_FORCE_INPUT) / 3.3) - 0.5;
                double sElevationA = (driverI.getAnalogIn(ANALOG_ELEVATION_INPUT) / 3.3) - 0.5;
                double leftStick = driveStick.getLeftThumbStickX();
                double rightStick = driveStick.getRightThumbStickX();

                if (game != null) {
                }
                // Control game mech
                boolean isShooting = false;
                if (game != null) {
                    ballLoaded = game.IsBallLoaded();
                }
//                System.out.println("Load in: " + game.isLoaderIn() + ",load out: " + game.isLoaderOut() + ",load up: " + game.isLoaderUp() + ",load down; " + game.isLoaderDown());
                if (shooterStick.getRightTrigger() > 0.5 && game != null) {
                    game.Shoot(drive, sForceA);
                    isShooting = true;
                    //System.out.println("Swish");
                } else if (shooterStick.getButtonA() && game != null) {
                    game.Aim(drive, sForceA);
                    isShooting = true;
                    // isDriving = true;
                } else if (shooterStick.getButtonRightBumper() && game != null) {
                    game.ReArmShooter();
                    isShooting = true;
                } else if (shooterStick.getButtonX() && game != null) {
                    game.DeployLoader();
                } else if (shooterStick.getButtonY() && game != null) {
                    game.RetractLoader();
                }

                if (shooterStick.getLeftTrigger() > 0.5 && game != null) {
                    game.ReloadShooter();
//                    isShooting = true;
                } else if (shooterStick.getButtonLeftBumper()) {
                    game.StartConveyor(Loader.CONVEYOR_DOWN_SPEED);
                } else {
                    game.StopConveyor();
                }

                if (!isShooting) {
                    double shooterRightStick = shooterStick.getRightThumbStickY();
                    if (shooterRightStick <= -1.0 * JOYSTICK_DEADBAND || shooterRightStick >= JOYSTICK_DEADBAND) {
                        if (game != null) {
                            game.AdjustShooterForce(shooterRightStick);
                        }
                    } else if (game != null) {
                        game.AdjustShooterForce(0.0);
                    }
                    double shooterLeftStick = shooterStick.getLeftThumbStickY();
                    if (shooterLeftStick <= -1.0 * JOYSTICK_DEADBAND || shooterLeftStick >= JOYSTICK_DEADBAND) {
                        if (game != null) {
                            game.AdjustShooterElevation(shooterLeftStick);
                        }
                    } else if (game != null) {
                        game.AdjustShooterElevation(0.0);
                    }
                }

                //Control drive
                boolean isDriving = false;
                if (driveStick.getButtonLeftBumper() && drive != null) {
                    isDriving = drive.RotateLeft(1.0);
                } else if (driveStick.getButtonRightBumper() && drive != null) {
                    isDriving = drive.RotateRight(1.0);
                } else if (driveStick.getButtonB() && drive != null) {
                    drive.FaceBackboard();
                    isDriving = true;
                }
//                else if (driveStick.getButtonB() && drive != null)
//                {
//                    isDriving = drive.GoToBall(game);
//                }
                if (!isDriving && drive != null) {
                    drive.DisableTurnController();
                    if (leftStick <= -1.0 * JOYSTICK_DEADBAND || leftStick >= JOYSTICK_DEADBAND) {
                        drive.CrabSteer(driveStick.getLeftThumbStickX(), driveStick.getTriggerAbsoluteValue());
                    } else if (rightStick <= -1.0 * JOYSTICK_DEADBAND || rightStick >= JOYSTICK_DEADBAND) {
                        drive.SlewSteer(driveStick.getRightThumbStickX(), driveStick.getTriggerAbsoluteValue());
                    } else {
                        drive.CrabSteer(0.0, driveStick.getTriggerAbsoluteValue());
                    }
                    if (driveStick.getLeftTrigger() >= JOYSTICK_DEADBAND) {
                        drive.Drive(driveStick.getAxisTrigger());
                    } else if (driveStick.getRightTrigger() >= JOYSTICK_DEADBAND) {
                        drive.Drive(driveStick.getAxisTrigger());
                    }
                }
                Timer.delay(PERIOD);
                getWatchdog().feed();
            }
            if (drive != null) {
                drive.disable();
            }
            if (game != null) {
                game.Disable();
            }
        } catch (CANTimeoutException ex) {
        }
    }
}