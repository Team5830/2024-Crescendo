// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DriveTeleop extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final SwerveDrive m_swerveDrive;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerveDrive The subsystem used by this command.
   */
  public DriveTeleop(
      SwerveDrive swerveDrive,
      SlewRateLimiter m_xspeedLimiter,
      SlewRateLimiter m_yspeedLimiter,
      SlewRateLimiter m_rotLimiter,
      DoubleSupplier xSpeed,
      DoubleSupplier ySpeed,
      DoubleSupplier rotSpeed,
      boolean fieldRelative,
      DoubleSupplier periodSeconds) {
    m_swerveDrive = swerveDrive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var x = -m_xspeedLimiter.calculate(xSpeed.getAsDouble()) * Constants.DriveTrain.maxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var y = -m_yspeedLimiter.calculate(ySpeed.getAsDouble()) * Constants.DriveTrain.maxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = -m_rotLimiter.calculate(rotSpeed.getAsDouble()) * Constants.DriveTrain.maxAngularVelocity;

    m_swerveDrive.drive(x, y, rot, fieldRelative, periodSeconds.getAsDouble());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
}
