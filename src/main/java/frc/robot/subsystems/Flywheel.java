// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase {
    CANSparkMax m_motor;
    SparkPIDController m_motorPID;
    RelativeEncoder m_encoder;
    public boolean isShooterOn = false;

    public double motorSpeed;

    public Flywheel() {
        m_motor = new CANSparkMax(Constants.Flywheel.motorChanel, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();

        m_motorPID = m_motor.getPIDController();
        m_encoder = m_motor.getEncoder();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Flywheel Speed ", m_encoder.getVelocity());
    }

    public boolean getShooterState() {
        return isShooterOn;
    }

    public void shooterOn() {
        m_motorPID.setReference(motorSpeed, ControlType.kVelocity);
        isShooterOn = true;
    }

    public void shooterGo() {
        m_motor.set(0.1);
        isShooterOn = true;
    }

    public void shooterOff() {
        m_motor.set(0);
        isShooterOn = false;
    }

    public boolean readyToShoot() {
        return (Math.abs(motorSpeed - m_encoder.getVelocity()) < Constants.Flywheel.speedTolerance);
    }
}
