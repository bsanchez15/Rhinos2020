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
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private CANSparkMax climber1 = new CANSparkMax(Constants.Climber1_ID, MotorType.kBrushless);
  /**
   * Creates a new Climber.
   */
  public Climber() {

  }

  public void setClimberOutput(double power){
    climber1.set(power*.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
