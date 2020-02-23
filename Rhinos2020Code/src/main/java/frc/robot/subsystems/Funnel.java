/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FunnelConstants;

public class Funnel extends SubsystemBase {
  private CANSparkMax Funnel1 = new CANSparkMax(FunnelConstants.Funnel1_ID, MotorType.kBrushed);
  private CANSparkMax Funnel2 = new CANSparkMax(FunnelConstants.Funnel2_ID, MotorType.kBrushed);
  /**
   * Creates a new Funnel.
   */
  public Funnel() {


  }

  public void setOutput(double speed ) {
    Funnel2.set(speed*.7);
    Funnel1.set(speed*.7);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
