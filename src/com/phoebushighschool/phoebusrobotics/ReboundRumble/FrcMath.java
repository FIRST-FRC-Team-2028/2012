/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package com.phoebushighschool.phoebusrobotics.ReboundRumble;

/**
 *
 * @author jmiller015
 */
public class FrcMath {
    
    public static double Abs(double input)
    {
        if (input < 0.0)
            return -1.0 * input;
        return input;
    }
        
    /***
     * Normalize()
     * 
     * this method reduces a number to 5 decimal places
     * 
     * @param num a very long decimal
     * @return 
     */
    static public String Normalize(double num)
    {
        int t = ((int)(num * 1000.0));
        double d = t / 1000.0;
        String str = "" + d;
        while (str.length() < 5)
            str += "0";
        return str;
        //return String.format("%10.4f", num);
    }
}
