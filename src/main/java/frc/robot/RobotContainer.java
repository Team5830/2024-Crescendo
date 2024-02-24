// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.subsystems.Intake;
import frc.robot.commands.*;
import frc.robot.commands.DriveTeleop;
import frc.robot.subsystems.SwerveDriveSub;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final XboxController m_controller = new XboxController(Constants.Joystick.port);
  
  private final SwerveDriveSub m_swerveDrive = new SwerveDriveSub(new File(Filesystem.getDeployDirectory(),"swerve"));


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
    SmartDashboard.putNumber("LeftX", m_controller.getLeftX());
    SmartDashboard.putNumber("LeftY", m_controller.getLeftY());
    SmartDashboard.putNumber("RightX", m_controller.getRightX());
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
    
    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = m_swerveDrive.driveCommand(
        () -> MathUtil.applyDeadband(m_controller.getLeftY(), Constants.Joystick.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_controller.getLeftX(), Constants.Joystick.LEFT_X_DEADBAND),
        () -> m_controller.getRightX(),
        () -> m_controller.getRightY());
    Command driveFieldOrientedDirectAngleSim = m_swerveDrive.simDriveCommand(
      () -> MathUtil.applyDeadband(m_controller.getLeftY(), Constants.Joystick.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(m_controller.getLeftX(), Constants.Joystick.LEFT_X_DEADBAND),
      () -> m_controller.getRawAxis(2));

    m_swerveDrive.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
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
    //Trigger button1 = m_controller.button(6);

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed, cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    m_controller.a().onTrue(new Positioning(m_arm, Constants.Arm.Position1.armAngle));
    m_controller.b().onTrue(new Positioning(m_arm, Constants.Arm.Position2.armAngle));
    m_controller.x().onTrue(new Positioning(m_arm, Constants.Arm.Position3.armAngle));
    m_controller.y().onTrue(new Positioning(m_arm, Constants.Arm.Position4.armAngle));
    m_controller.button(8).onTrue(new InstantCommand(m_intake::startFirstIntake));
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
