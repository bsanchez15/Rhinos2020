/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
//IDs
public static final int Controller1RightX = 4;
public static final int Controller1RightY = 5;
public static final int Controller1LeftY = 1;
public static final int Controller1LeftX = 0;


	public static final int ShooterFW1_ID = 1;
	public static final int ShooterFW2_ID = 2;
	
	public static final int Controller1ID = 1;


//Field/Robot Dimensions
	public static final double LL_ANGLE = 0;
	public static final double INNER_HEIGHT = 0;
	public static final double LL_HEIGHT = 0;


//Functions
	public static final int DvsV_FirstTerm = 0;
	public static final int DvsV_SecondTerm = 0;

//Gains
	public static final double Shooter_ks = 0.0183; 
	public static final double Shooter_kv= 0.121;
	
	public static final double Drive_ksVolts = 0.039;
	public static final double Drive_kvVoltsSecondPerMeter = 2.01;
	public static final double Drive_kaVoltsSecondsSquaredPerMeter = 0.204;

	public static final double Drive_kP = 8.41;

//Ramsete Parameters
	public static final double kRamseteB = 2;
	public static final double kRamseteZeta = 0.7;

	public static final double DGWheelCircInches = 6*Math.PI;
	public static final double DGWheelCircMeters =Units.inchesToMeters(6*Math.PI);
	public static final boolean kGyroReversed = false;
	public static final double DriveMaxVelocity = 0;
	public static final double DriveMaxAcceleration = 0;
	public static final double kaVoltSecondsSquaredPerMeter = 0;



	public static final class DriveConstants {
		public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics (Units.inchesToMeters(26.75));


		public static final int lMasterID = 1;
		public static final int RMasterID = 3;
		public static final int lSlaveID = 2;
		public static final int rSlaveID = 4;
		public static final int DriveLeftShifter1ID = 0;
		public static final int DriveleftShifter2ID = 0;


		public static final double DriveGearLowReduction = 1/7.29;
		public static final double DriveGearHighReduction = 1/15;
	}

	public static final class IntakeConstants {
		public static final int Intake_Shifter2_1 = 0;
		public static final int Intake_Shifter2_2 = 0;

		public static final int Intake_Shifter1_1 = 0;
		public static final int Intake_Shifter1_2 = 1;
		public static final int Intake1_ID = 9;
		public static final int Intake2_ID = 10;
	}

}

