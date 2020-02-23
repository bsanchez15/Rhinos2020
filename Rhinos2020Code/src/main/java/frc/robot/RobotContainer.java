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
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ShootAlign;
import frc.robot.commands.ShootRange;
import frc.robot.commands.TestShoot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Funnel;
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
 
  SlewRateLimiter leftLimiter = new SlewRateLimiter(0.5);
  SlewRateLimiter rightLimiter = new SlewRateLimiter(0.5);
  // The robot's subsystems and commands are defined here...
  XboxController controller1 = new XboxController(Constants.Controller1ID);
  XboxController controller2 = new XboxController(Constants.Controller2ID);
  private final DriveTrain m_Drive = new DriveTrain();
  private final Shooter m_Shooter = new Shooter();
  private final Intake m_Intake = new Intake();
  private final Funnel m_Funnel = new Funnel();
  private final Climber m_Climber = new Climber();

  private final ShootRange m_ShootRange = new ShootRange(m_Shooter);
  private final ShootAlign m_ShootAlign = new ShootAlign(m_Drive);
  private final TestShoot m_TestShoot = new TestShoot(m_Shooter);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {


   /* m_Intake.setDefaultCommand(new RunCommand(()-> 
    m_Intake.setIntakeOutput((controller2.getRawAxis(Constants.Controller2RightTriggerID)-(controller2.getRawAxis(Constants.Controller2LeftTrigger)))), m_Intake));
    ;*/
    //Default Commands
    m_Drive.setDefaultCommand(new RunCommand(()-> 
    m_Drive.ArcadeDrive(leftLimiter.calculate(controller1.getRawAxis(Constants.Controller1LeftY)*.4) ,rightLimiter.calculate(controller1.getRawAxis(Constants.Controller1RightX)*.4)),m_Drive));
    // Configure the button bindings
 

    m_Funnel.setDefaultCommand(new RunCommand(()-> 
    m_Funnel.setOutput(controller2.getRawAxis(Constants.Controller2LeftY)), m_Funnel));

    configureButtonBindings();

    m_Climber.setDefaultCommand(new RunCommand(()-> 
    m_Climber.setClimberOutput(controller2.getRawAxis(Constants.Controller2RightY)), m_Climber));

    ;

  


  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //Solenoid bindings
    new JoystickButton(controller2, Button.kA.value)
    .whenPressed(new RunCommand(() -> m_Intake.setShifterForward(), m_Intake));
new JoystickButton(controller2, Button.kB.value)
    .whenPressed(new RunCommand(() -> m_Intake.setShifterReverse(), m_Intake));
    

    new JoystickButton(controller1, Button.kA.value)
        .whenPressed(new RunCommand(() -> m_Drive.setHighGear(), m_Drive));
    new JoystickButton(controller1, Button.kB.value)
        .whenPressed(new RunCommand(() -> m_Drive.setLowGear(), m_Drive));

        //Limelight Commands bindings

    new JoystickButton(controller1, Button.kY.value).whenPressed(m_ShootRange);
    new JoystickButton(controller1, Button.kX.value).whileHeld(m_ShootAlign);
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
      DriverStation.reportError("Unable to open trajectory1!", ioe.getStackTrace());
    }

    if (trajectory1 != null){
    RamseteCommand ramseteCommandFirstRoute = new RamseteCommand(
        trajectory1,
        m_Drive::getPose,
        new RamseteController(2, .7),
        m_Drive.getFeedforward(),
        m_Drive.getKinematics(),
        m_Drive::getSpeeds,
        m_Drive.getLeftPIDController(),
        m_Drive.getRightPIDController(),
        m_Drive::setOutputVolts,
        m_Drive
    );

   // Trajectory trajectory2 = null;

   /* try {
      trajectory2 = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/paths/SimpleTurn.wpilib.json"));
    }
    catch (IOException ioe) { 
      DriverStation.reportError("Unable to open trajectory2!", ioe.getStackTrace());
    }*/


    return new SequentialCommandGroup(ramseteCommandFirstRoute, m_ShootAlign );
    //return ramseteCommandFirstRoute.andThen(() -> m_Drive.setOutputVolts(0, 0));
    } else {
      return null;

    
    
  }
    



    // An ExampleCommand will run in autonomous
    
  
  }
}
