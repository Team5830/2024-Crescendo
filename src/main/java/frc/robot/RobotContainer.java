// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj.shuffleboard.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final XboxController m_controller = new XboxController(Constants.Joystick.port);
  
  private final SwerveDrive m_swerveDrive = new SwerveDrive();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(Constants.Joystick.xRateLimit);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(Constants.Joystick.yRateLimit);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Constants.Joystick.rotRateLimit);

  private DoubleSupplier getPeriod;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(DoubleSupplier getPeriod) {
    this.getPeriod = getPeriod;

    /*SmartDashboard.putData("Position1" new Positioning(m_armangle, Position1.armangle))
      SmartDashboard.putData("Position2" new Positioning(m_armangle, Position2.armangle))
      SmartDashboard.putData("Pickup" new Pickup(m_intake))
      SmartDashboard.putData("Shoot" new Shoot(m_flywheel))
      SmartDashboard.putData("AutonomousCommandC" new AutonomousCommandC(m_flywheel, m_drivetrain, m_intake, m_arm))
      SmartDashboard.putData("AutonomousCommandR" new AutonomousCommandR(m_flywheel, m_drivetrain, m_intake, m_arm))
      SmartDashboard.putData("AutonomousCommandL" new AutonomousCommandL(m_flywheel, m_drivetrain, m_intake, m_arm))
      SmartDashboard.putData("AutonomousCommandA" new AutonomousCommandA(m_flywheel, m_drivetrain, m_intake, m_arm)) */
    //Configure the trigger bindings
    SmartDashboard.putNumber("LeftX", m_controller.getLeftX());
    SmartDashboard.putNumber("LeftY", m_controller.getLeftY());
    SmartDashboard.putNumber("RightX", m_controller.getRightX());
    SmartDashboard.putNumber("DriveP", Constants.DriveTrain.driveControllerKp);
    SmartDashboard.putNumber("DriveI", Constants.DriveTrain.driveControllerKi);
    SmartDashboard.putNumber("DriveD", Constants.DriveTrain.driveControllerKd);
    SmartDashboard.putNumber("Angle Tolerance", Constants.DriveTrain.AngleTolerance);
    configureBindings();
    SmartDashboard.putData("TestTurn", new testTurning(m_swerveDrive));
     m_swerveDrive.setDefaultCommand(new DriveTeleop(
        m_swerveDrive,
        m_xspeedLimiter,
        m_yspeedLimiter,
        m_rotLimiter,
        m_controller::getLeftX,
        m_controller::getLeftY,
        m_controller::getRightX,
        true,
        this.getPeriod));
        
  }
    
  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
   * constructor with an arbitrary predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new DriveTeleop(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed, cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*public Command getAutonomousCommand() {
    // An example command will be run in autonomous
  }*/
}
