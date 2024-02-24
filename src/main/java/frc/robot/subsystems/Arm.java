package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import com.revrobotics.CANSparkBase.ControlType;

public class Arm extends SubsystemBase {
  private CANSparkMax m_motor;
  public RelativeEncoder m_encoder;
  private SparkPIDController m_pidController;
  private double targecko;

  public Arm() {
    try {
      m_motor = new CANSparkMax(Constants.Arm.motorChanel, CANSparkMax.MotorType.kBrushless);
      m_motor.restoreFactoryDefaults();
      m_encoder = m_motor.getEncoder();
      m_encoder.setPositionConversionFactor(8);
      m_encoder.setPosition(0.0);
      m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
      m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
      m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.Arm.forwardLimit);
      m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, Constants.Arm.reverseLimit);

      m_pidController = m_motor.getPIDController();
      m_pidController.setP(Constants.Arm.kP);
      m_pidController.setI(Constants.Arm.kI);
      m_pidController.setD(Constants.Arm.kD);
      m_pidController.setFF(Constants.Arm.kFF);
      m_pidController.setOutputRange(Constants.Arm.minOutput, Constants.Arm.maxOutput);
      move(0);
      // m_karmoterPID.setPositionPIDWrappingMaxInput(180);
      // m_karmoterPID.setPositionPIDWrappingMinInput(-180);
      // m_karmoterPID.setPositionPIDWrappingEnabled(true);

    } catch (RuntimeException ex) {
      DriverStation.reportError("Error configuring arm: " + ex.getMessage(), true);
    }

  }

  public void move(double degrees) {
    targecko = degrees;
    m_pidController.setReference(targecko, ControlType.kPosition);
    DriverStation.reportWarning(String.format("Armm Position %f", m_encoder.getPosition()), false);
  }

  public boolean AtTarget() {
    double curposition = m_encoder.getPosition();
    DriverStation.reportWarning(String.format("Position: %f", curposition), false);
    if (Math.abs(curposition - targecko) <= Constants.Arm.tolerance) {
      DriverStation.reportWarning("True", false);
      return true;
    } else {
      DriverStation.reportWarning(String.format("false: %f", Math.abs(curposition - targecko)), false);
      return false;
    }
  }

  public double Position() {
    SmartDashboard.putNumber("Armzzz", m_encoder.getPosition());
    SmartDashboard.putNumber("AFL", Constants.Arm.forwardLimit);
    SmartDashboard.putNumber("ALR", Constants.Arm.reverseLimit);
    return m_encoder.getPosition();
  }

  public boolean Safe() {
    if (Position() > 200 || AtTarget()) {
      return true;
    } else {
      return false;
    }
  }

  public void Stop() {
    // armMotorController.set(0);
    m_motor.stopMotor();
    SmartDashboard.putNumber("ArmPosition", m_encoder.getPosition());
  }

  public void increment() {
    if (targecko + 3 <= Constants.Arm.forwardLimit) {
      targecko = targecko + 3;
      m_pidController.setReference(targecko, ControlType.kPosition);
    }
  }

  public void decrement() {
    if (targecko - 3 >= Constants.Arm.reverseLimit) {
      targecko = targecko - 3;
      m_pidController.setReference(targecko, ControlType.kPosition);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ArmPosition", m_encoder.getPosition());
    // SmartDashboard.getNumber("karget", karget );
  }
}