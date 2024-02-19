// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;

public class SwerveModule {
  private CANSparkMax m_driveMotor;
  private CANSparkMax m_turningMotor;
  private SparkAbsoluteEncoder m_angleEncoder;
  private RelativeEncoder m_positionEncoder;
  private boolean invertencoder = false;
  private boolean invertmotor = false;
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
   * and turning encoder,
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      boolean invertencoder, 
      boolean invertmotor, 
      double zerooffset) {
    try {
      m_driveMotor = new CANSparkMax(driveMotorChannel, CANSparkMax.MotorType.kBrushless);
      m_turningMotor = new CANSparkMax(turningMotorChannel, CANSparkMax.MotorType.kBrushless);

      m_driveMotor.restoreFactoryDefaults();
      m_turningMotor.restoreFactoryDefaults();
      m_driveMotor.setIdleMode(IdleMode.kCoast);
      m_turningMotor.setIdleMode(IdleMode.kBrake);
      m_driveMotor.setInverted(invertmotor);
      m_positionEncoder = m_driveMotor.getEncoder();
      m_angleEncoder = m_turningMotor.getAbsoluteEncoder(Type.kDutyCycle);
      m_positionEncoder.setPositionConversionFactor(12.5 * 2.54 / 6.55 / 100);
      m_angleEncoder.setPositionConversionFactor(2*Math.PI);
      if (invertencoder == true){
         m_angleEncoder.setInverted(true);
      }
      m_angleEncoder.setZeroOffset(zerooffset);
      m_turningPIDController.enableContinuousInput(0, 2*Math.PI);
      
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating swerve module: " + ex.getMessage(), true);
    }
    
    
  }
  
  public void setTurnTarget(double setpoint){
    m_turningPIDController.setGoal(setpoint);
  }
  
  public void updatePIDValues(double P, double I, double D){
   m_turningPIDController.setPID(P,I,D); 

  }

  public double getPValue(){
   return m_turningPIDController.getP(); 
  }

  public double getIValue(){
   return m_turningPIDController.getI(); 
  }
  public double getDValue(){
   return m_turningPIDController.getD(); 
  }
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveMotor.getAbsoluteEncoder(Type.kDutyCycle).getVelocity(),
        new Rotation2d(invertencoder ? -m_angleEncoder.getPosition() : m_angleEncoder.getPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition(),
        new Rotation2d(m_angleEncoder.getPosition()));
  }

  public double Angle(){
    //return m_angleEncoder.getPosition()*(180/Math.PI)-180; //(0 to 2PI)*(180/PI)-180
    return m_angleEncoder.getPosition();
  }

  public double Offset(){
    return m_positionEncoder.getPosition();
  }

  public double AngleMotorCurrent(){
    return m_turningMotor.getOutputCurrent();
  }
  
  public double DriveMotorCurrent(){
    return m_driveMotor.getOutputCurrent();
  }
  
  public double AngleMotorVoltage(){
    return m_turningMotor.getBusVoltage();
  }
  
  public double DriveMotorVoltage(){
    return m_driveMotor.getBusVoltage();
  }

  public double Goal(){
    return m_turningPIDController.getGoal().position;
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
