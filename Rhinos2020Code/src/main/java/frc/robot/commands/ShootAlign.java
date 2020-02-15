/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.limelightvision.LimeLight;
import frc.robot.subsystems.DriveTrain;

public class ShootAlign extends CommandBase {
  private static final double THRESHOLD = 0;
double steeringadjust;
  double threshold;
  /**
   * Creates a new ShootAlign.
   */
  DriveTrain m_DriveTrain = new DriveTrain();

  public ShootAlign(DriveTrain D) {
m_DriveTrain = D;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double Kp = Constants.Drive_kP;
    double tx = m_DriveTrain.getlimelight().getdegRotationToTarget();
    boolean isTargetSeen = m_DriveTrain.getlimelight().getIsTargetFound();
    double error = -tx;


    if ((tx > 1)& isTargetSeen){
      steeringadjust = Kp * error - threshold;
    }

    else if ((tx < 1) & isTargetSeen){
      steeringadjust = Kp * error + threshold;
    }
    m_DriveTrain.tankDriveSimpleTeleop(
      steeringadjust, 
      -steeringadjust);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(m_DriveTrain.getlimelight().getdegRotationToTarget()) < THRESHOLD) {
      return true;
    } 
    else return false;
  }
}
