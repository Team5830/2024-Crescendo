// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;

import com.revrobotics.SparkAbsoluteEncoder.Type;
import frc.robot.Constants;

public class SwerveModule {
  private CANSparkMax m_driveMotor;
  private CANSparkMax m_turningMotor;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(
      Constants.DriveTrain.driveControllerKp,
      Constants.DriveTrain.driveControllerKi,
      Constants.DriveTrain.driveControllerKd);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      Constants.DriveTrain.turnControllerKp,
      Constants.DriveTrain.turnControllerKi,
      Constants.DriveTrain.turnControllerKd,
      new TrapezoidProfile.Constraints(
          Constants.DriveTrain.maxAngularVelocity, Constants.DriveTrain.maxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(
      Constants.DriveTrain.driveFeedforwardStatic, Constants.DriveTrain.driveFeedforwardVelocity);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(
      Constants.DriveTrain.turnFeedforwardStatic, Constants.DriveTrain.turnFeedforwardVelocity);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
   * and turning encoder.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel) {
    try {
      m_driveMotor = new CANSparkMax(driveMotorChannel, CANSparkMax.MotorType.kBrushless);
      m_turningMotor = new CANSparkMax(turningMotorChannel, CANSparkMax.MotorType.kBrushless);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating swerve module: " + ex.getMessage(), true);
    }

    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveMotor.getAbsoluteEncoder(Type.kDutyCycle).getVelocity(),
        new Rotation2d(m_turningMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition(),
        new Rotation2d(m_turningMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(m_turningMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition());

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular
    // to the desired direction of travel that can occur when modules change
    // directions. This results in smoother driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(
        m_driveMotor.getAbsoluteEncoder(Type.kDutyCycle).getVelocity(),
        state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(
        m_turningMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition(),
        state.angle.getRadians());

    final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
  }
}
