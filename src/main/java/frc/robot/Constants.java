// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //drivetrain
    public static final int frontLeft = 1;
    public static final int frontRight = 2;
    public static final int rearLeft = 3;
    public static final int rearRight = 4;

    // Collector

    public static final int collectorMotor = 8;
    public static final int collectorSolenoid = 5;

    // Transfer

    public static final int transferSides = 10;
    public static final int transferFront = 11;
    public static final int transferRear = 12;


        //we still need to check the conversion later 
    public static final double encoderInchConversion = 14500 / 18.8; // ticks per inch


    public static final double encoderMeterConversion = (14500 / 0.47752) * .1; // ticks per 100ms as meters
    // same as inches just converted it to meters
	public static final int TOFRight = 11;
	public static final int TOFLeft = 10;


    //elevator 
            
     public static final int elevator1 =13;
     public static final int elevator2 = 14;
     


    //climber
       



}
