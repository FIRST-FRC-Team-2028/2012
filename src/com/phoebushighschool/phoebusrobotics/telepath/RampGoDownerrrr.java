///*
// * To change this template, choose Tools | Templates
// * and open the template in the editor.
// */
//
//package com.phoebushighschool.phoebusrobotics.telepath;
//import edu.wpi.first.wpilibj.Solenoid;
//import edu.wpi.first.wpilibj.DigitalInput;
///**
// *
// * @author Administrator
// */
//public class RampGoDownerrrr 
//{
//    Solenoid pushOutLeft;
//    Solenoid pullInLeft;
//    Solenoid pushOutRight;
//    Solenoid pullInRight;
////    DigitalInput DownerrrrOutLeft;
////    DigitalInput DownerrrrInLeft;
////    DigitalInput DownerrrrOutRight;
////    DigitalInput DownerrrrInRight;
//    
//    public RampGoDownerrrr()
//    {
//        pushOutLeft = new Solenoid(ReboundRumble.LOADER_OUT_SOLENOID_CHANNEL);
//        pullInLeft = new Solenoid(ReboundRumble.LOADER_IN_SOLENOID_CHANNEL);
//        pushOutRight = new Solenoid(ReboundRumble.LOADER_DOWN_SOLENOID_CHANNEL);
//        pullInRight = new Solenoid(ReboundRumble.LOADER_UP_SOLENOID_CHANNEL);
////        DownerrrrOutLeft = new DigitalInput(ReboundRumble.LOAD_OUT_GPIO_CHANNEL);
////        DownerrrrInLeft = new DigitalInput(ReboundRumble.LOADER_IN_SOLENOID_CHANNEL);
////        DownerrrrOutRight = new DigitalInput(ReboundRumble.LOAD_DOWN_GPIO_CHANNEL);
////        DownerrrrInRight = new DigitalInput(ReboundRumble.LOAD_UP_GPIO_CHANNEL);
//        pushOutLeft.set(false);
//        pullInLeft.set(true);
//        pushOutRight.set(false);
//        pullInRight.set(true);
//    }
//
//    /***
//     * Informs you if the RampGoDownerrrr is out.
//     * @return true - the RampGoDownerrrr is out
//     *         false - the RampGoDownerrrr is not out
//     */
////    public boolean isRampGoDownerrrrOut()
////    {
////        if (!DownerrrrOutLeft.get() && !DownerrrrOutRight.get())
////            return true;
////        return false;
////    }
//
//    /***
//     * Informs you if the RampGoDownerrrr is in.
//     * @return true - the RampGoDownerrrr is in
//     *         false - the RampGoDownerrrr is not in
//     */
////    public boolean isRampGoDownerrrrIn()
////    {
////        if (!DownerrrrInLeft.get() && !DownerrrrInRight.get())
////            return true;
////        return false;
////    }
//
//    /***
//     * Pushes the RampGoDownerrrr out.
//     * @return true - the RampGoDownerrrr is out
//     *         false - the RampGoDownerrrr is not out
//     */
//    public void PushRampGoDownerrrrOut()
//    {
//        pullInLeft.set(false);
//        pushOutLeft.set(true);
//        pullInRight.set(false);
//        pushOutRight.set(true);
////        return isRampGoDownerrrrOut();
//    }
//
//    /***
//     * Pushes the RampGoDownerrrr in.
//     * @return true - the RampGoDownerrrr is in
//     *         false - the RampGoDownerrrr is not in
//     */
//    public void PullRampGoDownerrrrIn()
//    {
//        pushOutLeft.set(false);
//        pullInLeft.set(true);
//        pushOutRight.set(false);
//        pullInRight.set(true);
////        return isRampGoDownerrrrIn();
//    }
//}
