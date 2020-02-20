package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.limelightvision.LimeLight;

public class DriveTrain extends SubsystemBase {

  LimeLight m_LimeLight = new LimeLight();

  private static final double kGearRatio = 7.29;
  private static final double kWheelRadiusInches = 3.0;

DoubleSolenoid leftLevelShifter = new DoubleSolenoid(DriveConstants.leftDriveShifter1ID, DriveConstants.leftDriveShifter2ID);
DoubleSolenoid rightLevelShifter = new DoubleSolenoid(DriveConstants.rightDriveShifter1ID, DriveConstants.rightDriveShifter2ID);


  CANSparkMax leftMaster = new CANSparkMax(DriveConstants.lMasterID, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightMaster = new CANSparkMax(DriveConstants.RMasterID, CANSparkMaxLowLevel.MotorType.kBrushless);

  CANSparkMax leftSlave = new CANSparkMax(DriveConstants.lSlaveID, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightSlave = new CANSparkMax(DriveConstants.rSlaveID, CANSparkMaxLowLevel.MotorType.kBrushless);

  ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(28));
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.3, 1.96, 0.06);

  PIDController leftPIDController = new PIDController(2.95, 0, 0);
  PIDController rightPIDController = new PIDController(2.95, 0, 0);

  Pose2d pose = new Pose2d();
  public DriveTrain() {
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftMaster.setInverted(false);
    rightMaster.setInverted(true);

    gyro.reset();
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        leftMaster.getEncoder().getVelocity() / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60,
        rightMaster.getEncoder().getVelocity() / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60
    );
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getPose() {
    return pose;
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  public PIDController getRightPIDController() {
    return rightPIDController;
  }

  public void setOutputVolts(double leftVolts, double rightVolts) {
    leftMaster.set(leftVolts / 12);
    rightMaster.set(rightVolts / 12);
  }

  public void reset() {
    odometry.resetPosition(new Pose2d(), getHeading());
  }

  @Override
  public void periodic() {
    pose = odometry.update(getHeading(), leftMaster.getEncoder().getPosition(), rightMaster.getEncoder().getPosition() );
  }


  public void tankDriveSimpleTeleop(double leftOutput, Double rightOutput){


    leftMaster.set((leftOutput));
    rightMaster.set((rightOutput));
  }

  public void GTADrive (double rotation, double forward, double reverse){
    leftMaster.set(forward-reverse+rotation);
    rightMaster.set(forward-reverse-rotation);
  }

  public void ArcadeDrive (double Ydisplacement, double rotation) {
    leftMaster.set(Ydisplacement + rotation);
    rightMaster.set(Ydisplacement - rotation);

    
  }

//  public double getHeading() {
//    return Math.IEEEremainder(gyro.getAngle(), 360) * (Constants.kGyroReversed ? -1.0 : 1.0);
  
//  }


  public void setLowGear (){
    leftLevelShifter.set(Value.kReverse);
    rightLevelShifter.set(Value.kReverse);
  }

  public void setHighGear (){
    leftLevelShifter.set(Value.kForward);
    rightLevelShifter.set(Value.kForward);

  }
  public void Log (){
    SmartDashboard.putNumber("tx", m_LimeLight.getdegRotationToTarget());
    SmartDashboard.putNumber("ty", m_LimeLight.getdegVerticalToTarget());
    SmartDashboard.putBoolean("TargetSeen", m_LimeLight.getIsTargetFound());
    SmartDashboard.putNumber(	"getHeading", gyro.getAngle());

  }


  public LimeLight getlimelight(){
    return m_LimeLight;
    
      }
  
}
