// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Climber;
import frc.robot.commands.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private final CommandXboxController xroller = new CommandXboxController(Constants.Joystick.port);
  private final SwerveDrive m_swerveDrive = new SwerveDrive();
  private final Arm m_arm = new Arm();
  private final Flywheel m_flywheel = new Flywheel();
  private final Intake m_intake = new Intake();
  private final Climber m_climber = new Climber();

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
      //SmartDashboard.putData("Shoot", new Shoot(m_flywheel));
    //Configure the trigger bindings
    SmartDashboard.putNumber("LeftX", xroller.getLeftX());
    SmartDashboard.putNumber("LeftY", xroller.getLeftY());
    SmartDashboard.putNumber("RightX", xroller.getRightX());
    SmartDashboard.putNumber("DriveP", Constants.DriveTrain.driveControllerKp);
    SmartDashboard.putNumber("DriveI", Constants.DriveTrain.driveControllerKi);
    SmartDashboard.putNumber("DriveD", Constants.DriveTrain.driveControllerKd);
    SmartDashboard.getNumber("TurnP", Constants.DriveTrain.turnControllerKp);
    SmartDashboard.getNumber("TurnI", Constants.DriveTrain.turnControllerKi);
    SmartDashboard.getNumber("TurnD", Constants.DriveTrain.turnControllerKd);
    SmartDashboard.putNumber("Turn Target", Constants.DriveTrain.turnarget);
    // Configure the trigger bindings
    configureBindings();

    SmartDashboard.putNumber("Angle Tolerance", Constants.DriveTrain.AngleTolerance);
    configureBindings();
    // SmartDashboard.putData("TestTurn", new testTurning(m_swerveDrive));
     m_swerveDrive.setDefaultCommand(new DriveTeleop(
        m_swerveDrive,
        m_xspeedLimiter,
        m_yspeedLimiter,
        m_rotLimiter,
        xroller::getLeftX,
        xroller::getLeftY,
        xroller::getRightX,
        false,
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
  // Create some buttons
  Trigger button1 = xroller.button(1);
  Trigger button2 = xroller.button(2);
  Trigger button3 = xroller.button(3);
  Trigger button4 = xroller.button(4);
  Trigger button5 = xroller.button(5);
  Trigger button6 = xroller.button(6);
  Trigger button7 = xroller.button(7);
  Trigger button8 = xroller.button(8);

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed, cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    button1.onTrue(new Positioning(m_arm, Constants.Arm.Position1.armAngle));
    button2.onTrue(new Positioning(m_arm, Constants.Arm.Position2.armAngle));
    button3.onTrue(new Positioning(m_arm, Constants.Arm.Position3.armAngle));
    button4.onTrue(new Positioning(m_arm, Constants.Arm.Position4.armAngle));
    xroller.povDown().and(button5).whileTrue(new InstantCommand(m_climber::lencerement).repeatedly());
    xroller.povDown().and(button6).whileTrue(new InstantCommand(m_climber::lencerement));
    xroller.povDown().and(button4).whileTrue(new InstantCommand(m_climber))
    button8.onTrue(new InstantCommand(m_intake::startFirstIntake));
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
