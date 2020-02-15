/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  DoubleSolenoid shifter1 = new DoubleSolenoid(Constants.Intake_Shifter1_1, Constants.Intake_Shifter1_2);
  DoubleSolenoid shifter2 = new DoubleSolenoid(Constants.Intake_Shifter2_1, Constants.Intake_Shifter2_2);


  /**
   * Creates a new Intake.
   */
  public Intake() {

  }

  public void setShifterForward() {
shifter1.set(Value.kForward);
  }
  public void setShifterReverse() {
    shifter1.set(Value.kReverse);
      }
    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
