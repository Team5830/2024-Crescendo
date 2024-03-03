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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase {
    CANSparkMax m_topMotor;
    CANSparkMax m_bottomMotor;
    SparkPIDController m_topMotorPID;
    SparkPIDController m_bottomPID;
    RelativeEncoder m_topEncoder;
    RelativeEncoder m_bottomEncoder;
    public boolean isShooterOn = false;

    public double motorSpeed = 7.0;
    public double halfspeed = Constants.flywheel.halfspeed;

    public Flywheel(){
        try {
            m_topMotor = new CANSparkMax(Constants.flywheel.motorChanelTop, MotorType.kBrushless);
            m_bottomMotor = new CANSparkMax(Constants.flywheel.motorChanelBottom, MotorType.kBrushless);

            m_topMotor.restoreFactoryDefaults();
            m_bottomMotor.restoreFactoryDefaults();
            m_topMotorPID = m_topMotor.getPIDController();
            m_bottomPID = m_bottomMotor.getPIDController();
            m_topEncoder = m_topMotor.getEncoder();
            m_bottomEncoder = m_bottomMotor.getEncoder();
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating flywheel: " + ex.getMessage(), true);
        }
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Top Flywheel Speed ", m_topEncoder.getVelocity());
        // SmartDashboard.putNumber("Bottom Flywheel Speed ", m_bottomEncoder.getVelocity());
    }

    public boolean getShooterState() {
        return isShooterOn;
    }

    public void shooterOn() {
        m_topMotorPID.setReference(motorSpeed, ControlType.kVelocity);
        m_bottomPID.setReference(motorSpeed, ControlType.kVelocity);
        isShooterOn = true;
    }
    public void shooterHalf(){
        m_topMotorPID.setReference(halfspeed, ControlType.kVelocity);
        m_bottomPID.setReference(halfspeed, ControlType.kVelocity);
        isShooterOn = true;
    }

    public void shooterGo() {
        m_topMotor.setVoltage(-9);
        m_bottomMotor.setVoltage(9);
        isShooterOn = true;
    }

    public void shooterOff() {
        m_topMotor.setVoltage(0);
        m_bottomMotor.setVoltage(0);
        isShooterOn = false;
    }

    public void shooterToggle() {
        if(isShooterOn){
            shooterOff();
        } else {
            shooterGo();
        }
    }

    public boolean readyToShoot() {
        return (Math.abs(motorSpeed - m_topEncoder.getVelocity()) < Constants.flywheel.speedTolerance && Math.abs(motorSpeed - m_bottomEncoder.getVelocity()) < Constants.flywheel.speedTolerance);
    }
}
