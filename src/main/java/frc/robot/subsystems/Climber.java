package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Utils;

import com.revrobotics.CANSparkBase.ControlType;

public class Climber extends SubsystemBase {

  private CANSparkMax m_leftMotor;
  public RelativeEncoder m_leftEncoder;
  private SparkPIDController m_leftPIDController;
  private double leftTarget;
  private CANSparkMax m_rightMotor;
  public RelativeEncoder m_rightEncoder;
  private SparkPIDController m_rightPIDController;
  private double rightTarget;

  public Climber() {
    try {
      m_leftMotor = new CANSparkMax(Constants.climber.leftMotorChanel, CANSparkMax.MotorType.kBrushless);
      m_leftMotor.restoreFactoryDefaults();
      m_leftEncoder = m_leftMotor.getEncoder();
      m_leftEncoder.setPositionConversionFactor(1);
      m_leftEncoder.setPosition(0.0);
      m_leftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
      m_leftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
      m_leftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.climber.upLeftHeight);
      m_leftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, Constants.climber.downLeftHeight - 5);

      m_rightMotor = new CANSparkMax(Constants.climber.rightMotorChanel, CANSparkMax.MotorType.kBrushless);
      m_rightMotor.restoreFactoryDefaults();
      m_rightEncoder = m_rightMotor.getEncoder();
      m_rightEncoder.setPositionConversionFactor(1);
      m_rightEncoder.setPosition(0.0);
      m_rightMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
      m_rightMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
      m_rightMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.climber.upRightHeight);
      m_rightMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, Constants.climber.downRightHeight - 5);

      m_leftPIDController = m_leftMotor.getPIDController();
      m_leftPIDController.setP(Constants.climber.kP);
      m_leftPIDController.setI(Constants.climber.kI);
      m_leftPIDController.setD(Constants.climber.kD);
      m_leftPIDController.setFF(Constants.climber.kFF);
      m_leftPIDController.setOutputRange(Constants.climber.minOutput, Constants.climber.maxOutput);
      leftMove(0);
      m_rightPIDController = m_rightMotor.getPIDController();
      m_rightPIDController.setP(Constants.climber.kP);
      m_rightPIDController.setI(Constants.climber.kI);
      m_rightPIDController.setD(Constants.climber.kD);
      m_rightPIDController.setFF(Constants.climber.kFF);
      m_rightPIDController.setOutputRange(Constants.climber.minOutput, Constants.climber.maxOutput);
      rightMove(0);
      // m_karmoterPID.setPositionPIDWrappingMaxInput(180);
      // m_karmoterPID.setPositionPIDWrappingMinInput(-180);
      // m_karmoterPID.setPositionPIDWrappingEnabled(true);

    } catch (RuntimeException ex) {
      DriverStation.reportError("Error configuring Climber: " + ex.getMessage(), true);
    }

  }

  public void leftMove(double height) {
    leftTarget = height;
    m_leftPIDController.setReference(leftTarget, ControlType.kPosition);
    DriverStation.reportWarning(String.format("Climberr Position %f", m_leftEncoder.getPosition()), false);
  }

  public void rightMove(double height) {
    rightTarget = height;
    m_rightPIDController.setReference(rightTarget, ControlType.kPosition);
    DriverStation.reportWarning(String.format("Climberrm Position %f", m_rightEncoder.getPosition()), false);
  }

  public boolean leftTarget() {
    double position = m_leftEncoder.getPosition();
    DriverStation.reportWarning(String.format("Position: %f", position), false);
    if (Math.abs(position - leftTarget) <= Constants.climber.tolerance) {
      DriverStation.reportWarning("True", false);
      return true;
    } else {
      DriverStation.reportWarning(String.format("false: %f", Math.abs(position - leftTarget)), false);
      return false;
    }
  }

  public boolean rightOnTarget() {
    double position = m_rightEncoder.getPosition();
    DriverStation.reportWarning(String.format("Position: %f", position), false);
    if (Math.abs(position - rightTarget) <= Constants.climber.tolerance) {
      DriverStation.reportWarning("True", false);
      return true;
    } else {
      DriverStation.reportWarning(String.format("false: %f", Math.abs(position - rightTarget)), false);
      return false;
    }
  }

  public double leftPosition() {
    SmartDashboard.putNumber("Climberzzz", m_leftEncoder.getPosition());
    // SmartDashboard.putNumber("AFL", Constants.climber.upHeight);
    // SmartDashboard.putNumber("ALR", Constants.climber.downHeight);
    return m_leftEncoder.getPosition();
  }

  public double rightPosition() {
    SmartDashboard.putNumber("Climberzzz", m_rightEncoder.getPosition());
    // SmartDashboard.putNumber("AFL", Constants.climber.upHeight);
    // SmartDashboard.putNumber("ALR", Constants.climber.downHeight);
    return m_rightEncoder.getPosition();
  }

  public boolean Safe() {
    // Always stay safe !
    return true;
  }

  public void rightStop() {
    // armMotorController.set(0);
    m_rightMotor.stopMotor();
    SmartDashboard.putNumber("ClimberPosition", m_rightEncoder.getPosition());
  }

  public void leftStop() {
    // armMotorController.set(0);
    m_leftMotor.stopMotor();
    SmartDashboard.putNumber("ClimberPosition", m_leftEncoder.getPosition());
  }

  public void leftChange(double value) {
    m_leftPIDController.setReference(Utils.clamp(
        value,
        (double) Constants.climber.downLeftHeight,
        (double) Constants.climber.upLeftHeight),
        ControlType.kPosition);
  }

  public void rightChange(double value) {
    m_leftPIDController.setReference(Utils.clamp(
        value,
        (double) Constants.climber.downRightHeight,
        (double) Constants.climber.upRightHeight),
        ControlType.kPosition);
  }

  public void useUpPosition() {
    leftMove(Constants.climber.upLeftHeight);
    rightMove(Constants.climber.upRightHeight);
  }

  public void useDownPosition() {
    leftMove(0);
    rightMove(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Climber Position", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Climber Position", m_rightEncoder.getPosition());
    SmartDashboard.getNumber("Left Target", leftTarget);
    SmartDashboard.getNumber("Right Target", rightTarget);
  }
}