/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class TestShoot extends CommandBase {
  
  Shooter m_shooter;
  MedianFilter tyfilter = new MedianFilter(10);
  /**
   * Creates a new ShootRange.
   */
  public TestShoot(Shooter S) {
    m_shooter = S;  
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double Output = 0;
    m_shooter.log();
    m_shooter.useOutput(0, Output);
    double ty = m_shooter.getlimelight().getdegVerticalToTarget();

    SmartDashboard.putNumber("Distance", DistanceCalc(ty)); 
    SmartDashboard.putNumber("Output", Output);   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  public double DistanceCalc (double a1){
    double a2 = Constants.LL_ANGLE;
    double h1 = Constants.INNER_HEIGHT;
    double h2 = Constants.LL_HEIGHT;

    return h1-h2/(Math.tan(a2+a1));

  }



  
}
