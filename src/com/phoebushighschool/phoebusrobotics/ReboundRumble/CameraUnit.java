/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package com.phoebushighschool.phoebusrobotics.ReboundRumble;

import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.image.NIVision.MeasurementType;
import edu.wpi.first.wpilibj.image.*;


/**
 *
 * @author jmiller015
 */
public class CameraUnit
{
    
    public static final int IMAGE_WIDTH = 320;
    public static final int IMAGE_HEIGHT = 240;
    public static final double TARGET_WIDTH = 2.0;
    public static final double TARGET_HEIGHT = 1.5;
    public static final double RECTANGLE_QUALITY_THRESHOLD = 95.0;
    public static final double PARTICLE_TO_IMAGE_THRESHOLD = 0.05;
    public static final double PARTICLE_AREA_THRESHOLD = 250.0;
    AxisCamera camera;
    Servo cameraPan;    
    Servo cameraTilt;    
    ColorImage cameraImage = null;
    BinaryImage threshold = null;
    BinaryImage onlyBigObjects = null;
    BinaryImage filledObjects = null;
    BinaryImage filteredObjects = null;
    ParticleAnalysisReport[] reports;
    ParticleAnalysisReport r = null;
    CriteriaCollection cc;
    MathUtils math;
//    Gyro gyro = new Gyro(ReboundRumble.ROBOT_ANGLE_GYRO_SENSOR);
    protected boolean xCentered = false;
    protected boolean yCentered = false;
    protected boolean bothCentered = false;
    protected static final double CENTER_DEADBAND = 0.3;
    protected double distance = 0.0;
    protected double elevationS = 10.0;
    protected double forceS = 0.0;
    protected double offset = 0.0;
    protected double feildOfVeiwWidth = 0.0;
    public double loadPosition = 160.0;
    public double neutralPosition = 85.0;
    public double neutralTiltPosition = 100.0;
    
    public CameraUnit()
    {
        camera = AxisCamera.getInstance(TwentyTwelve.CAMERA_IP);
        camera.writeResolution(AxisCamera.ResolutionT.k320x240);
        camera.writeExposurePriority(AxisCamera.ExposurePriorityT.imageQuality);
        camera.writeExposureControl(AxisCamera.ExposureT.hold);
        camera.writeWhiteBalance(AxisCamera.WhiteBalanceT.fixedIndoor);
        cc = new CriteriaCollection();
        cc.addCriteria(MeasurementType.IMAQ_MT_BOUNDING_RECT_WIDTH, 15, 400, false);
        cc.addCriteria(MeasurementType.IMAQ_MT_BOUNDING_RECT_HEIGHT, 15, 400, false);
        cameraPan = new Servo(1, 6);
        cameraTilt = new Servo(1, 7);
    }

    /**
     * this method will take a picture with the camera and find the highest
     * target in the camera's field of view
     *
     * @return true - a target was found false - no target was found
     */
    public boolean FindTarget()
    {
        boolean targetFound = false;
        try
        {
//            double time = Timer.getFPGATimestamp();
            cameraImage = camera.getImage();
            threshold = cameraImage.thresholdHSV(110, 150, 200, 255, 240, 255);
            onlyBigObjects = threshold.removeSmallObjects(false, 1);
            filledObjects = onlyBigObjects.convexHull(false);
            filteredObjects = filledObjects.particleFilter(cc);
            reports = filteredObjects.getOrderedParticleAnalysisReports();
            
//            System.out.println("Find Target took: " + (Timer.getFPGATimestamp() - time));
            // determine which of the blobs in reports[] is actually the highest target
            r = null;
            for (int i = 0; i < reports.length; i++)
            {
                //System.out.println("target #" + (i + 1) + ": (" + reports[i].boundingRectLeft + ", " + reports[i].boundingRectTop + ", " + reports[i].boundingRectWidth + ", " + reports[i].boundingRectHeight + "), Q=" + reports[i].particleQuality + ", A=" + reports[i].particleArea + ", %=" + reports[i].particleToImagePercent);
                if (reports[i].particleQuality >= RECTANGLE_QUALITY_THRESHOLD
                        && reports[i].particleToImagePercent >= PARTICLE_TO_IMAGE_THRESHOLD
                        && reports[i].particleArea >= PARTICLE_AREA_THRESHOLD)
                {
                    if (r == null)
                    {
                        r = reports[i];
                    } else if (r.boundingRectLeft > reports[i].boundingRectLeft)
                    {
                        r = reports[i];
                    }
                }
                
            }

//             determine the center of mass of the highest report

            //move the camera up/down, and/or left/right to put the center of
            // mass of the highest report in the exact center of the image
            if (r != null)
            {
                MoveCameraToTarget();
//                distance = GetDistanceToTarget(r.boundingRectHeight);
//                System.out.println("\tDistance to target:  " + ((int) distance / 12) + "\'" + (int) distance % 12 + "\"");
//                System.out.println("Offset to the target: " + getAngleToTarget());
//                System.out.println("Servo interpratation: " + ConvertAngleToServoAngle(getAngleToTarget()));
//                System.out.println("Servo Angle x: " + cameraPan.getAngle() + "servo angle y: " + cameraTilt.getAngle());
                targetFound = true;
            } else
            {
                SetNeutralPosition();
//                System.out.println("\tNo target found");
            }
            
            cameraImage.free();
            threshold.free();
            onlyBigObjects.free();
            filledObjects.free();
            filteredObjects.free();
        } catch (AxisCameraException ex)
        {
        } catch (NIVisionException ex)
        {
        }
        
        return targetFound;
    }
    
    public int Abs(int value)
    {
        if (value >= 0)
        {
            return value;
        }
        return -1 * value;
    }

    /**
     * Return distance to target in inches
     *
     * @param targetHeightInPixels
     * @return
     */
    public double GetDistanceToTarget(int targetHeightInPixels)
    {
        double fieldOfViewHeight = TARGET_HEIGHT / targetHeightInPixels * IMAGE_HEIGHT;
        distance = (fieldOfViewHeight / 2.0) / /*
                 * tan 23.5 degrees
                 */ 0.4348123749;
                 /*tan 17.625 degrees
                  * 0.3176989915;
                  */
        return distance * 12.0 * 1.4;
    }
    
    public double GetDistance()
    {
        return distance;
    }

    /**
     * Given the highest target in the camera's field of view, compute the
     * elevation the SlingShot needs to be set at to score a goal.
     *
     * @return double - degrees of elevation for the SlingShot (in the range 0.0
     * ... 45.0 degrees)
     */
    public double GetShooterElevation()
    {
        // $$$ ToDo:  Calculate elevationS (shooter elevation) here, based on the distance
        return ((45 / 360) + 0.5);
    }

    /**
     * Compute force the SlingShot must be set at to score a goal when fired at
     * the highest target in the camera's field of view to score a goal,
     * assuming the SlingShot is at the correct elevation.
     *
     * @return double - percentage of SlingShot force in the range 0.0 ...
     * 100.0%
     */
    public double GetShooterForce(double distance)
    {
        if (GetDistance() > 100)
            return 100;
        return GetDistance();
    }
    
    public void MoveCameraToTarget()
    {
        double temp;
        if (r != null)
        {
            cameraPan.setAngle(cameraPan.getAngle());
            if (getAngleToTarget() > -85 && getAngleToTarget() < 85)
            {
                if (getAngleToTarget() <= 0.05 && getAngleToTarget() >= -0.05)
                {
                    cameraPan.setAngle(cameraPan.getAngle());
                    xCentered = true;
                } else if (getAngleToTarget() > 0.05 && getAngleToTarget() < 85)
                {
                    temp = ((cameraPan.getAngle() + getAngleToTarget()) * 0.995);
                    if (temp < 10.0)
                        cameraPan.setAngle(10.0);
                    else
                        cameraPan.setAngle(temp);
                    xCentered = true;
                } else if (getAngleToTarget() < -0.05 && getAngleToTarget() > -85)
                {
                    temp = ((cameraPan.getAngle() + getAngleToTarget()) * 1.005);
                    if (temp > 160.0)
                        cameraPan.setAngle(160.0);
                    else 
                        cameraPan.setAngle(temp);
                    xCentered = true;
                }
                
            }
            cameraTilt.setAngle(cameraTilt.getAngle());
            if (getAngleToTarget() > -85 && getAngleToTarget() < 85)
            {
                if (getElavation() <= 0.05 && getElavation() >= -0.05)
                {
                    cameraTilt.setAngle(cameraTilt.getAngle());
                    yCentered = true;
                } else if (getElavation() > 0.05 && getElavation() < 85)
                {
                    temp = ((cameraTilt.getAngle() + getElavation()) * 0.995);
                    if (temp < 10.0)
                        cameraTilt.setAngle(10.0);
                    else 
                        cameraTilt.setAngle(temp);
                    yCentered = true;
                } else if (getElavation() < -0.05 && getElavation() > -85)
                {
                    temp = ((cameraTilt.getAngle() + getElavation()) * 1.005);
                    if (temp > 160.0)
                        cameraTilt.setAngle(160.0);
                    else 
                        cameraTilt.setAngle(temp);
                    yCentered = true;
                }
                
            }
        } else        
        {
            cameraPan.setAngle(neutralPosition);
            cameraTilt.setAngle(neutralPosition);
        }
    }

    public void SetLoadPosition()
    {
        cameraPan.setAngle(neutralPosition);
        cameraTilt.setAngle(loadPosition);
    }
    
    public void SetNeutralPosition()
    {
        cameraPan.setAngle(neutralPosition);
        cameraTilt.setAngle(neutralTiltPosition);
    }
    /**
     * Compute the relative angle to the highest target in the camera's field of
     * view. this uses the robot's current heading as 0.0 degrees, with positive
     * output being right (clockwise) and negative being left
     * (counter-clockwise).
     *
     * @return double - angle to the target in degrees, the robot must turn this
     * amount to score a goal
     */
    public double getAngleToTarget()
    {
        if (r != null)
        {
            offset = r.center_mass_x - (IMAGE_WIDTH / 2);
            offset = MathUtils.atan(offset / 367.97488075578);
        }
        return ConvertRadiansToDegrees(offset);
    }
    
    public double getElavation()
    {
        if (r != null)
        {
            offset = r.center_mass_y - (IMAGE_HEIGHT / 2);
            offset = MathUtils.atan(offset / 377.71602429076);
        }
        return ConvertRadiansToDegrees(offset);
    }
    
    public double ConvertRadiansToDegrees(double radians)
    {
        return (radians * 180.0) / 3.1415926535898;
    }
    
    public double ConvertAngleToServoAngle(double a)
    {
        if (a > -85 && a < 85)
        {
            if (a > -85 && a < -0.05)
            {
                a = a - (a * 2);
                return a;
            } else
            {
                a = a - (a * 2);
                return a;
            }
        }
        return 85;
    }
}
