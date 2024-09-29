// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final Vision m_vision = new Vision();
  private final Arm m_arm = new Arm();
  private final Flywheel m_flywheel = new Flywheel();
  private final Intake m_intake = new Intake();
  private final Climber m_climber = new Climber();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xSpeedLimiter = new SlewRateLimiter(Constants.controller.xRateLimit);
  private final SlewRateLimiter m_ySpeedLimiter = new SlewRateLimiter(Constants.controller.yRateLimit);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Constants.controller.rotRateLimit);

  private final DoubleSupplier getPeriod;

  private SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(DoubleSupplier getPeriod) {
    this.getPeriod = getPeriod;
    // Configure the trigger bindings
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
    //Add Screen Buttons
    SmartDashboard.putData("Intake Position",new Positioning(m_arm, Constants.arm.positionIntake));
    SmartDashboard.putData("Shoot Position",new Positioning(m_arm, Constants.arm.positionShoot));
    SmartDashboard.putData("Upright Position",new Positioning(m_arm, Constants.arm.positionUpright));
    SmartDashboard.putData("Shoot",new Shoot(m_flywheel,m_intake) );
    SmartDashboard.putData("reset gyro",new InstantCommand(m_swerveDrive::resetHeading));
    //SmartDashboard.putData("Pickup",new Pickup );
    // Configure the trigger and button bindings
    configureBindings();

    m_swerveDrive.setDefaultCommand(new DriveTeleop(
        m_swerveDrive,
        m_xSpeedLimiter,
        m_ySpeedLimiter,
        m_rotLimiter,
        () -> flyskyController.getRawAxis(0),
        () -> -flyskyController.getRawAxis(1),
        () -> -flyskyController.getRawAxis(3) / 2,
        false,
        this.getPeriod));
    // m_vision.setDefaultCommand(new InstantCommand(m_vision::periodic));

    m_chooser.addOption("Auto Amp", new AutonomousCommandAmp(m_swerveDrive, m_intake, m_flywheel));
    m_chooser.addOption("Auto Speaker", new AutonomousCommandSpeaker(m_swerveDrive, m_intake, m_flywheel, m_arm));
    SmartDashboard.putData(m_chooser);
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
    // xboxController.povLeft().onTrue( new LineUpForAmp(m_flywheel, m_intake, m_swerveDrive, m_vision));
    // xboxController.povRight().toggleOnTrue( new AimAtSpeaker(m_flywheel, m_intake, m_swerveDrive, m_vision,m_arm));
      /*new SequentialCommandGroup(
         new AimAtSpeaker(m_flywheel, m_intake, m_swerveDrive, m_vision)
         new InstantCommand(m_flywheel::shooterGo),
         new WaitCommand(0.5),
         new InstantCommand(m_intake::startFirstIntake),
         new WaitUntilCommand(m_intake::noteSensorIsNotDetected),
         new WaitCommand(0.5),
         new InstantCommand(m_intake::stopFirstIntake),
         new InstantCommand(m_flywheel::shooterOff))); */
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new DriveTeleop(m_exampleSubsystem));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed, cancelling on release.

     xboxController.a().onTrue(new Positioning(m_arm, Constants.arm.positionIntake));
     xboxController.b().onTrue(new Positioning(m_arm, Constants.arm.positionShoot));
     xboxController.y().onTrue(new Positioning(m_arm, Constants.arm.positionUpright));
     xboxController.rightBumper().onTrue(new Positioning(m_arm, Constants.arm.positionAmp));
    //  xboxController.povDown().onTrue(new AimAtSpeaker(m_flywheel, m_intake, m_swerveDrive, m_vision,m_arm));
    // xboxController.rightBumper()
    //     .onTrue(new ClimberLeveling(m_climber, m_swerveDrive).deadlineWith(new WaitCommand(10)));
    xboxController.povUp().onTrue(new InstantCommand(m_climber::useUpPosition));
    xboxController.povDown().onTrue(new InstantCommand(m_climber::useDownPosition));

    // xboxController.axisGreaterThan(1, Constants.controller.climberAxesThreshold).whileTrue(new InstantCommand(
    //     () -> m_climber.leftChange(-Constants.controller.climberAxesMultiplier * xboxController.getRawAxis(1))));
    // xboxController.axisLessThan(1, -Constants.controller.climberAxesThreshold).whileTrue(new InstantCommand(
    //     () -> m_climber.leftChange(-Constants.controller.climberAxesMultiplier * xboxController.getRawAxis(1))));
    // xboxController.axisGreaterThan(5, Constants.controller.climberAxesThreshold).whileTrue(new InstantCommand(
    //     () -> m_climber.rightChange(-Constants.controller.climberAxesMultiplier * xboxController.getRawAxis(5))));
    // xboxController.axisLessThan(5, -Constants.controller.climberAxesThreshold).whileTrue(new InstantCommand(
    //     () -> m_climber.rightChange(-Constants.controller.climberAxesMultiplier * xboxController.getRawAxis(5))));

    xboxController.leftTrigger().toggleOnTrue(new IntakeCommand(m_intake));
    xboxController.back().onTrue(new IntakeEject(m_intake));
    xboxController.rightTrigger().toggleOnTrue(new Shoot(m_flywheel,m_intake));
    xboxController.x().toggleOnTrue(new ShootAmp(m_flywheel, m_intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
