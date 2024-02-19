// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;
import com.revrobotics.CANSparkBase.ControlType;

public class SwerveModule {
  private CANSparkMax m_driveMotor;
  private CANSparkMax m_turningMotor;
  private SparkAbsoluteEncoder m_angleEncoder;
  private RelativeEncoder m_positionEncoder;
  private boolean invertencoder = false;
  private boolean invertmotor = false;
  private double target;
  // Gains are for example purposes only - must be determined for your own robot!
  private SparkPIDController m_drivePIDController;
  /*  = new PIDController(
      Constants.DriveTrain.driveControllerKp,
      Constants.DriveTrain.driveControllerKi,
      Constants.DriveTrain.driveControllerKd); */
  private SparkPIDController m_turningPIDController; 
  // Gains are for example purposes only - must be determined for your own robot!
  /* 
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      Constants.DriveTrain.turnControllerKp,
      Constants.DriveTrain.turnControllerKi,
      Constants.DriveTrain.turnControllerKd,
      new TrapezoidProfile.Constraints(
          Constants.DriveTrain.maxAngularVelocity, Constants.DriveTrain.maxAngularAcceleration));
*/
  
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
      m_drivePIDController = m_driveMotor.getPIDController();
      m_turningPIDController = m_turningMotor.getPIDController();
      
      m_driveMotor.restoreFactoryDefaults();
      m_turningMotor.restoreFactoryDefaults();
      m_driveMotor.setIdleMode(IdleMode.kCoast);
      m_turningMotor.setIdleMode(IdleMode.kBrake);
      m_driveMotor.setInverted(invertmotor);
      m_positionEncoder = m_driveMotor.getEncoder();
      m_drivePIDController.setFeedbackDevice(m_positionEncoder);
      m_angleEncoder = m_turningMotor.getAbsoluteEncoder(Type.kDutyCycle);
      m_positionEncoder.setPositionConversionFactor(12.5 * 2.54 / 6.55 / 100);
      m_angleEncoder.setPositionConversionFactor(360);
      if (invertencoder == true){
         m_angleEncoder.setInverted(true);
      }
      m_angleEncoder.setZeroOffset(zerooffset);
      m_turningPIDController.setFeedbackDevice(m_angleEncoder);
      m_turningPIDController.setPositionPIDWrappingMaxInput(359);
      m_turningPIDController.setPositionPIDWrappingMinInput(0);
      m_turningPIDController.setPositionPIDWrappingEnabled(true);
      //m_turningPIDController.setSmartMotionMaxVelocity(Constants.DriveTrain.maxAngularVelocity,0);
      //m_turningPIDController.setSmartMotionAccelStrategy(0);
      m_drivePIDController.setFeedbackDevice(m_positionEncoder);
      updateTurnPIDValues(Constants.DriveTrain.turnControllerKp,Constants.DriveTrain.turnControllerKi,Constants.DriveTrain.turnControllerKd);
      updateDrivePIDValues(Constants.DriveTrain.driveControllerKp,Constants.DriveTrain.driveControllerKi,Constants.DriveTrain.driveControllerKd);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating swerve module: " + ex.getMessage(), true);
    }
    m_angleEncoder.getPosition();
  }
  
  public void setTurnTarget(double setpoint){
    target = setpoint;
    m_turningPIDController.setReference(setpoint,ControlType.kPosition);
  }
  
  public void updateTurnPIDValues(double P, double I, double D){
   m_turningPIDController.setP(P);
   m_turningPIDController.setI(I); 
   m_turningPIDController.setD(D);
  }
  public void updateDrivePIDValues(double P, double I, double D){
   m_drivePIDController.setP(P);
   m_drivePIDController.setI(I); 
   m_drivePIDController.setD(D);
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
  public boolean atTarget(){
    if (Math.abs(m_angleEncoder.getPosition() - target)< Constants.DriveTrain.AngleTolerance){
      return true;
    }else{
      return false;
    }
  }
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveMotor.getAbsoluteEncoder(Type.kDutyCycle).getVelocity(),
        new Rotation2d(m_angleEncoder.getPosition()));
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

  }
  public void PIDStop(){
    m_turningMotor.stopMotor();
  }

}
