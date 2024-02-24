// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase {
    CANSparkMax m_topmoter;
    CANSparkMax m_bottomoter;
    SparkPIDController m_topmotorPID;
    SparkPIDController m_bottomPID;
    RelativeEncoder m_topencoder;
    RelativeEncoder m_bottomEncoder;
    public boolean isShooterOn = false;

    public double motorSpeed = 7.0;

    public Flywheel(){
        try {
            m_topmoter = new CANSparkMax(Constants.Flywheel.motorChanel, MotorType.kBrushless);
            m_bottomoter = new CANSparkMax(Constants.Flywheel.motorChanel, MotorType.kBrushless);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating flywheel: " + ex.getMessage(), true);
        }

        m_topmoter.restoreFactoryDefaults();
        m_bottomoter.restoreFactoryDefaults();
        m_topmotorPID = m_topmoter.getPIDController();
        m_bottomPID = m_bottomoter.getPIDController();
        m_topencoder = m_topmoter.getEncoder();
        m_bottomEncoder = m_bottomoter.getEncoder();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Top Flywheel Speed ", m_topencoder.getVelocity());
        SmartDashboard.putNumber("Bottom Flywheel Speed ", m_bottomEncoder.getVelocity());
    }

    public boolean getShooterState() {
        return isShooterOn;
    }

    public void shooterOn() {
        m_topmotorPID.setReference(motorSpeed, ControlType.kVelocity);
        m_bottomPID.setReference(motorSpeed, ControlType.kVelocity);
        isShooterOn = true;
    }

    public void shooterGo() {
        m_topmoter.setVoltage(0.1);
        m_bottomoter.setVoltage(-0.1);
        isShooterOn = true;
    }

    public void shooterOff() {
        m_topmoter.setVoltage(0);
        m_bottomoter.setVoltage(0);
        isShooterOn = false;
    }

    public boolean readyToShoot() {
        return (Math.abs(motorSpeed - m_topencoder.getVelocity()) < Constants.Flywheel.speedTolerance && Math.abs(motorSpeed - m_bottomEncoder.getVelocity()) < Constants.Flywheel.speedTolerance);
    }
}
