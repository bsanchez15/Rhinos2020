/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
//  private DoubleSolenoid intakePiston1 = new DoubleSolenoid(IntakeConstants.Intake_Shifter1_1, IntakeConstants.Intake_Shifter1_2);
//  private DoubleSolenoid intakePiston2 = new DoubleSolenoid(IntakeConstants.Intake_Shifter2_1, IntakeConstants.Intake_Shifter2_2);

  private CANSparkMax intake1 = new CANSparkMax(IntakeConstants.Intake1_ID, MotorType.kBrushed);
  private CANSparkMax intake2 = new CANSparkMax(IntakeConstants.Intake2_ID, MotorType.kBrushed);
//Comunicacion no sirve y Paolo no viene :)


  /**
   * Creates a new Intake.
   */
  public Intake() {
    intake1.restoreFactoryDefaults();
    intake2.restoreFactoryDefaults();

    intake1.setIdleMode(IdleMode.kCoast);
    intake2.setIdleMode(IdleMode.kCoast);


  }

  /*public void setShifterForward() {
intakePiston1.set(Value.kForward);
intakePiston2.set(Value.kForward);

  }
  public void setShifterReverse() {
    intakePiston1.set(Value.kReverse);
    intakePiston2.set(Value.kReverse);

      }*/

  public void setIntakeOutput (double power){
    intake1.set(power);
    intake2.set(power);
  }
    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
