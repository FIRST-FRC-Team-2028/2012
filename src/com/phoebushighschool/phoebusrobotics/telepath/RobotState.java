/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package com.phoebushighschool.phoebusrobotics.telepath;

/**
 *
 * @author jmiller015
 */
public class RobotState
{
    public static final int Aim = 1;
    public static final int Shoot = 2;
    public static final int Reload = 3;
    
    int state;
    
    public RobotState()
    {
        state = Aim;
    }
    
    public int GetState()
    {
        return state;
    }
    
    public void NextState()
    {
        if (state == Reload)
            state = Aim;
        else
            state++;
    }
    public String toString()
    {
        String temp = "RobotState: ";
        switch (state)
        {
            case 1:
                temp += "Aim";
                break;
            case 2:
                temp += "Shoot";
                break;
            case 3:
                temp += "Reload";
                break;
        }
        return temp;
    }
}
