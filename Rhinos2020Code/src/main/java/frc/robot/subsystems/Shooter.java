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

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.limelightvision.LimeLight;

public class Shooter extends PIDSubsystem {
  CANSparkMax flywheel1 = new CANSparkMax(Constants.ShooterFW1_ID, MotorType.kBrushless);
  CANSparkMax flywheel2 = new CANSparkMax(Constants.ShooterFW2_ID, MotorType.kBrushless);
  CANEncoder encoder = new CANEncoder(flywheel1);

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Shooter_ks, Constants.Shooter_kv);

  LimeLight m_limelight;
  

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));
  }

  @Override
  public void useOutput(double output, double setpoint) {
    flywheel1.follow(flywheel2, true);
    flywheel1.setVoltage(output+feedforward.calculate(setpoint));
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }

  public LimeLight getlimelight(){
return m_limelight;

  }

  public void log() {
    SmartDashboard.putBoolean("TargetFound[tv]", getlimelight().getIsTargetFound());
    SmartDashboard.putNumber("GetDegVertical[ty]", getlimelight().getdegVerticalToTarget());
    SmartDashboard.putNumber("GetDegHorizontal[tx]", getlimelight().getdegRotationToTarget());
  }
}
