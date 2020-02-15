/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.limelightvision.LimeLight;

public class DriveTrain extends SubsystemBase {
  private LimeLight m_limelight;

 
  //private DoubleSolenoid leftLevelShifter = new DoubleSolenoid(DriveConstants.DriveLeftShifter1ID, DriveConstants.DriveleftShifter2ID);
  //private DoubleSolenoid rightLevelShifter = new DoubleSolenoid(DriveConstants.DriveLeftShifter1ID, DriveConstants.DriveleftShifter2ID);

 private CANSparkMax leftMaster = new CANSparkMax(DriveConstants.lMasterID,MotorType.kBrushless);
 private CANSparkMax rightMaster = new CANSparkMax(DriveConstants.RMasterID,MotorType.kBrushless);

private CANSparkMax leftSlave = new CANSparkMax(DriveConstants.lSlaveID, MotorType.kBrushless);
private CANSparkMax rightSlave = new CANSparkMax(DriveConstants.rSlaveID, MotorType.kBrushless);

private DifferentialDrive m_drive = new DifferentialDrive(leftMaster, rightMaster);

private CANEncoder leftEncoder = new CANEncoder(leftMaster);
private CANEncoder rightEncoder = new CANEncoder(rightMaster);

public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics (Units.inchesToMeters(26.75));

Gyro gyro = new ADXRS450_Gyro(SPI.Port.kMXP);

private DifferentialDriveOdometry m_Odometry;
  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftMaster.setInverted(false);
    rightMaster.setInverted(true);

    leftEncoder.setPositionConversionFactor(Constants.DGWheelCircMeters/DriveConstants.DriveGearHighReduction);
    rightEncoder.setPositionConversionFactor(Constants.DGWheelCircMeters/DriveConstants.DriveGearHighReduction);

    m_Odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }


  

  @Override
  public void periodic() {
    m_Odometry.update(Rotation2d.fromDegrees(getHeading()), leftEncoder.getPosition(), rightEncoder.getPosition());
    // This method will be called once per scheduler run
  }

  public Pose2d getPose () {
    return m_Odometry.getPoseMeters();
  }
  public DifferentialDriveWheelSpeeds getSpeeds(){
    return new DifferentialDriveWheelSpeeds(
    leftMaster.getEncoder().getVelocity()/DriveConstants.DriveGearHighReduction*Constants.DGWheelCircInches ,
    rightMaster.getEncoder().getVelocity()/DriveConstants.DriveGearHighReduction*Constants.DGWheelCircInches
    );
  }

  public DifferentialDriveKinematics getKinematics(){
    return kDriveKinematics;
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    m_Odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));

  }

  public void tankDriveVolts (double leftVolts, double rightVolts){
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(rightVolts);
  }

  public void tankDriveSimpleTeleop(double leftOutput, Double rightOutput){
    leftMaster.set(leftOutput*leftOutput*leftOutput);
    rightMaster.set(rightOutput*rightOutput*rightOutput);
  }

  public void GTADrive (double rotation, double forward, double reverse){
    leftMaster.set(forward-reverse+rotation);
    rightMaster.set(forward-reverse-rotation);
  }

  public void ArcadeDrive (double Ydisplacement, double rotation) {
    leftMaster.set(Ydisplacement + rotation);
    rightMaster.set(Ydisplacement - rotation);
  }

  
    
  public void resetEncoders(){
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

  }

  public double getAverageEncoderDistance() {
    return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
  }

  public CANEncoder getLeftEncoder(){
    return leftEncoder;
  }

  public CANEncoder getRightEncoder(){
    return rightEncoder;
  }   

  public void setMaxOutput(double maxOutput){
m_drive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360) * (Constants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getTurnRate() {
    return gyro.getRate() * (Constants.kGyroReversed ? -1.0 : 1.0);
  }

/*  public void setLowGear (){
    leftLevelShifter.set(DoubleSolenoid.Value.kReverse);
    rightLevelShifter.set(DoubleSolenoid.Value.kReverse);
  }

  public void setHighGear (){
    leftLevelShifter.set(DoubleSolenoid.Value.kForward);
    rightLevelShifter.set(DoubleSolenoid.Value.kForward);
  }*/

  public LimeLight getlimelight(){
    return m_limelight;
    
      }
  
}
