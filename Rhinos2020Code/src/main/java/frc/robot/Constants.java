/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final int ShooterFW1_ID = 1;
    public static final int ShooterFW2_ID = 2;
    public static final double Shooter_ks = 0.0183; 
    public static final double Shooter_kv= 0.121;
	public static final int DvsV_FirstTerm = 0;
	public static final int DvsV_SecondTerm = 0;
	public static final double LL_ANGLE = 0;
	public static final double INNER_HEIGHT = 0;
	public static final double LL_HEIGHT = 0;
	public static final int Controller1ID = 1;
}
