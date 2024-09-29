package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

public class Arm extends SubsystemBase {
  private CANSparkMax m_motor;
  public RelativeEncoder m_encoder;
  private SparkPIDController m_pidController;
  private double target;

  private final ArmFeedforward m_feedforward = new ArmFeedforward(Constants.arm.feedforwardKs, Constants.arm.feedforwardKv, Constants.arm.feedforwardKg);

  public Arm() {
    try {
      m_motor = new CANSparkMax(Constants.arm.motorChanel, CANSparkMax.MotorType.kBrushless);
      m_motor.restoreFactoryDefaults();
      m_encoder = m_motor.getEncoder();
      m_encoder.setPositionConversionFactor(4.4);
      m_encoder.setPosition(0.0);
      m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
      m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
      m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.arm.forwardLimit);
      m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, Constants.arm.reverseLimit);
      m_motor.setIdleMode(IdleMode.kBrake);

      m_pidController = m_motor.getPIDController();
      m_pidController.setP(Constants.arm.kP);
      m_pidController.setI(Constants.arm.kI);
      m_pidController.setD(Constants.arm.kD);
      m_pidController.setOutputRange(Constants.arm.minOutput, Constants.arm.maxOutput);
      move(0);
      // m_karmoterPID.setPositionPIDWrappingMaxInput(180);
      // m_karmoterPID.setPositionPIDWrappingMinInput(-180);
      // m_karmoterPID.setPositionPIDWrappingEnabled(true);

    } catch (RuntimeException ex) {
      DriverStation.reportError("Error configuring arm: " + ex.getMessage(), true);
    }

  }

  public void move(double degrees) {
    target = degrees;
    var feedforward = m_feedforward.calculate(Rotation2d.fromDegrees(degrees+90).getRadians(), 0);
    m_pidController.setFF(feedforward);
    m_pidController.setReference(target, ControlType.kPosition);
    DriverStation.reportWarning(String.format("Armm Position %f", m_encoder.getPosition()), false);
  }

  public boolean atTarget() {
    double currentPosition = m_encoder.getPosition();
    DriverStation.reportWarning(String.format("Position: %f", currentPosition), false);
    if (Math.abs(currentPosition - target) <= Constants.arm.tolerance) {
      DriverStation.reportWarning("True", false);
      return true;
    } else {
      DriverStation.reportWarning(String.format("false: %f", Math.abs(currentPosition - target)), false);
      return false;
    }
  }

  public double Position() {
    SmartDashboard.putNumber("Armzzz", m_encoder.getPosition());
    SmartDashboard.putNumber("AFL", Constants.arm.forwardLimit);
    SmartDashboard.putNumber("ALR", Constants.arm.reverseLimit);
    return m_encoder.getPosition();
  }

  public boolean Safe() {
    //Always stay safe !
      return true;
    }

  public void Stop() {
    // armMotorController.set(0);
    m_motor.stopMotor();
    SmartDashboard.putNumber("ArmPosition", m_encoder.getPosition());
  }

  public void increment() {
    if (target + Constants.arm.incrementValue <= Constants.arm.forwardLimit) {
      move(target + Constants.arm.incrementValue);
    }
  }

  public void decrement() {
    if (target - Constants.arm.incrementValue >= Constants.arm.reverseLimit) {
      move(target - Constants.arm.incrementValue);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ArmPosition", m_encoder.getPosition());
    SmartDashboard.putNumber("Arm voltage", m_motor.getAppliedOutput());
    // SmartDashboard.getNumber("target", target );
  }
}