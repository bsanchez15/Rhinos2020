/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ShootAlign;
import frc.robot.commands.ShootRange;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  XboxController controller1 = new XboxController(Constants.Controller1ID);
  private final DriveTrain m_robotDrive = new DriveTrain();
  private final Shooter m_Shooter = new Shooter();
  //private final Intake m_Intake = new Intake();

  private final ShootRange m_ShootRange = new ShootRange(m_Shooter);
  private final ShootAlign m_ShootAlign = new ShootAlign(m_robotDrive);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    m_robotDrive.setDefaultCommand(new RunCommand(()-> 
    
    m_robotDrive.ArcadeDrive(controller1.getRawAxis(Constants.Controller1LeftY)*.4,controller1.getRawAxis(Constants.Controller1RightX)*.4),m_robotDrive));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
 /*   new JoystickButton(controller1, Button.kA.value)
        .whenPressed(new InstantCommand(() -> m_Intake.setShifterForward()));
    new JoystickButton(controller1, Button.kB.value)
        .whenPressed(new InstantCommand(() -> m_Intake.setShifterReverse()));
*/
    new JoystickButton(controller1, Button.kY.value).whenPressed(m_ShootRange);
  
    new JoystickButton(controller1, Button.kX.value).whenPressed(m_ShootAlign);
}
    

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.Drive_ksVolts,
                                  Constants.Drive_kvVoltsSecondPerMeter,
                                  Constants.Drive_kaVoltsSecondsSquaredPerMeter), 
                                  DriveConstants.kDriveKinematics, 10);

    TrajectoryConfig config =
    new TrajectoryConfig(Constants.DriveMaxVelocity, Constants.DriveMaxAcceleration)
    .setKinematics(DriveConstants.kDriveKinematics)
    .addConstraint(autoVoltageConstraint);

    Trajectory trajectory1 = null;

    try {
      trajectory1 = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/paths/SimpleTurn.wpilib.json"));
    }
    catch (IOException ioe) { 
      DriverStation.reportError("Unable to open trajectory!", ioe.getStackTrace());
    }

    if (trajectory1 != null){
    RamseteCommand ramseteCommandFirstRoute = new RamseteCommand(
        trajectory1,
        m_robotDrive::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.Drive_ksVolts,
                                   Constants.Drive_kvVoltsSecondPerMeter,
                                   Constants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        m_robotDrive::getSpeeds,
        new PIDController(Constants.Drive_kP, 0, 0), new PIDController(Constants.Drive_kP, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts,
        m_robotDrive
    );
    return ramseteCommandFirstRoute.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    } else {
      return null;
    
  }
    



    // An ExampleCommand will run in autonomous
    
  
  }
}
