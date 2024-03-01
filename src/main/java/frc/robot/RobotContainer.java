// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
  private final CommandXboxController xboxController = new CommandXboxController(Constants.controller.xboxPort);
  private final GenericHID flyskyController = new GenericHID(Constants.controller.flyskyPort);
  private final SwerveDrive m_swerveDrive = new SwerveDrive();
  private final Arm m_arm = new Arm();
  private final Flywheel m_flywheel = new Flywheel();
  private final Intake m_intake = new Intake();
  private final Climber m_climber = new Climber();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xSpeedLimiter = new SlewRateLimiter(Constants.controller.xRateLimit);
  private final SlewRateLimiter m_ySpeedLimiter = new SlewRateLimiter(Constants.controller.yRateLimit);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Constants.controller.rotRateLimit);

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
      // SmartDashboard.putData("Shoot", new Shoot(m_flywheel));
    //Configure the trigger bindings
    SmartDashboard.putNumber("LeftX", xboxController.getLeftX());
    SmartDashboard.putNumber("LeftY", xboxController.getLeftY());
    SmartDashboard.putNumber("RightX", xboxController.getRightX());
    SmartDashboard.putNumber("DriveP", Constants.DriveTrain.driveControllerKp);
    SmartDashboard.putNumber("DriveI", Constants.DriveTrain.driveControllerKi);
    SmartDashboard.putNumber("DriveD", Constants.DriveTrain.driveControllerKd);
    SmartDashboard.getNumber("TurnP", Constants.DriveTrain.turnControllerKp);
    SmartDashboard.getNumber("TurnI", Constants.DriveTrain.turnControllerKi);
    SmartDashboard.getNumber("TurnD", Constants.DriveTrain.turnControllerKd);
    SmartDashboard.putNumber("Turn Target", Constants.DriveTrain.turnTarget);
    // Configure the trigger bindings
    configureBindings();

    SmartDashboard.putNumber("Angle Tolerance", Constants.DriveTrain.AngleTolerance);
    configureBindings();
   
    // SmartDashboard.putData("TestTurn", new testTurning(m_swerveDrive));
     m_swerveDrive.setDefaultCommand(new DriveTeleop(
        m_swerveDrive,
        m_xSpeedLimiter,
        m_ySpeedLimiter,
        m_rotLimiter,
        ()->flyskyController.getRawAxis(0),
        ()->-flyskyController.getRawAxis(1),
        ()->flyskyController.getRawAxis(3)/2,
        // xboxController::getLeftX,
        // xboxController::getLeftY,
        // xboxController::getRightX,
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

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed, cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    xboxController.a().onTrue(new Positioning(m_arm, Constants.arm.positionIntake));
    xboxController.b().onTrue(new Positioning(m_arm, Constants.arm.positionShoot));
    xboxController.y().onTrue(new Positioning(m_arm, Constants.arm.positionUpright));
    xboxController.rightBumper().onTrue(new ClimberLeveling(m_climber, m_swerveDrive).deadlineWith(new WaitCommand(10)));
    xboxController.povUp().onTrue(new InstantCommand(m_climber::useUpPosition));
    xboxController.povDown().onTrue(new InstantCommand(m_climber::useDownPosition));

    xboxController.axisGreaterThan(1,Constants.controller.climberAxesThreshold).whileTrue(new InstantCommand(()->m_climber.leftChange(-Constants.controller.climberAxesMultiplier*xboxController.getRawAxis(1))));
    xboxController.axisLessThan(1,-Constants.controller.climberAxesThreshold).whileTrue(new InstantCommand(()->m_climber.leftChange(-Constants.controller.climberAxesMultiplier*xboxController.getRawAxis(1))));
    xboxController.axisGreaterThan(5,Constants.controller.climberAxesThreshold).whileTrue(new InstantCommand(()->m_climber.rightChange(-Constants.controller.climberAxesMultiplier*xboxController.getRawAxis(5))));
    xboxController.axisLessThan(5,-Constants.controller.climberAxesThreshold).whileTrue(new InstantCommand(()->m_climber.rightChange(-Constants.controller.climberAxesMultiplier*xboxController.getRawAxis(5))));

    xboxController.leftTrigger().onTrue(new SequentialCommandGroup(
      new InstantCommand(m_intake::startFirstIntake),
      new WaitUntilCommand(m_intake::noteSensorIsDetected),
      new InstantCommand(m_intake::reverseFirstIntake),
      new WaitCommand(0.2),
      new InstantCommand(m_intake::stopFirstIntake)
    ));
    xboxController.back().onTrue(new SequentialCommandGroup(
      new InstantCommand(m_intake::reverseFirstIntake),
      new WaitCommand(1),
      new InstantCommand(m_intake::stopFirstIntake)
    ));
    // xboxController.leftBumper().onFalse(new InstantCommand(m_intake::stopFirstIntake));
    xboxController.rightTrigger().onTrue(new SequentialCommandGroup(
      new InstantCommand(m_flywheel::shooterGo),
      new WaitCommand(1),
      new InstantCommand(m_intake::startFirstIntake),
      new WaitUntilCommand(m_intake::noteSensorIsNotDetected),
      new WaitCommand(0.5),
      new InstantCommand(m_intake::stopFirstIntake),
      new InstantCommand(m_flywheel::shooterOff)
    ));
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
