// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSub;
import frc.robot.subsystems.Vision;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DriveTeleop extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final SwerveDriveSub m_swerveDrive;
  Vision vision;
  SlewRateLimiter m_xSpeedLimiter;
  SlewRateLimiter m_ySpeedLimiter;
  SlewRateLimiter m_rotLimiter;
  DoubleSupplier xSpeed;
  DoubleSupplier ySpeed;
  DoubleSupplier rotSpeed;
  BooleanSupplier enableVisionMovement;
  boolean fieldRelative;
  DoubleSupplier periodSeconds;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerveDrive The subsystem used by this command.
   */
  
  public DriveTeleop(
      SwerveDriveSub swerveDrive,
      SlewRateLimiter m_xspeedLimiter,
      SlewRateLimiter m_yspeedLimiter,
      SlewRateLimiter m_rotLimiter,
      DoubleSupplier xSpeed,
      DoubleSupplier ySpeed,
      DoubleSupplier rotSpeedX,
      DoubleSupplier rotSpeedY,
      boolean fieldRelative,
      DoubleSupplier periodSeconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);

    this.swerveDrive=swerveDrive;
    this.m_xspeedLimiter=m_xspeedLimiter;
    this.m_yspeedLimiter=m_yspeedLimiter;
    this.m_rotLimiter=m_rotLimiter;
    this.xSpeed=xSpeed;
    this.ySpeed=ySpeed;
    this.rotSpeedX=rotSpeedX;
    this.rotSpeedT=rotSpeedY;
    this.fieldRelative=fieldRelative;
    this.periodSeconds=periodSeconds;
 }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
    double rot = -m_rotLimiter.calculate(rotSpeed.getAsDouble()) * Constants.DriveTrain.maxAngularVelocity;

    SmartDashboard.putNumber("swerve: x", x);
    SmartDashboard.putNumber("swerve: y", y);
    SmartDashboard.putNumber("swerve: r", rot);
    swerveDrive.drive(x, y, rot, fieldRelative,false);
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
