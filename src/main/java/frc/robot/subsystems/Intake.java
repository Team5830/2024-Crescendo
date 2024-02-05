// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    public boolean intakeON = false;
    public boolean intakeReversed = false;

    CANSparkMax m_motor;
    SparkPIDController m_pidController;
    RelativeEncoder m_encoder;

    public Intake() {
        try {
            m_motor = new CANSparkMax(Constants.Intake.motorChanel, MotorType.kBrushless);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating intake: " + ex.getMessage(), true);
        }

        m_motor.restoreFactoryDefaults();
        m_pidController = m_motor.getPIDController();
        m_encoder = m_motor.getEncoder();
        m_encoder.setPosition(0.0);

        // set PID coefficients
        m_pidController.setP(Constants.Intake.P);
        m_pidController.setI(Constants.Intake.I);
        m_pidController.setD(Constants.Intake.D);
        m_pidController.setIZone(Constants.Intake.zI);
        m_pidController.setFF(Constants.Intake.F);
        m_pidController.setOutputRange(Constants.Intake.kMinOutput, Constants.Intake.kMaxOutput);
    }

    public void stopIntake() {
        m_motor.set(0);
    }

    public void startFirstIntake() {
        m_motor.set(-Constants.Intake.firstIntakSspeed);
        intakeON = true;
    }

    public void reverseFirstIntake() {
        m_motor.set(Constants.Intake.firstIntakSspeed);
        intakeON = true;
        intakeReversed = true;
    }

    public void stopFirstIntake() {
        m_motor.set(0);
        intakeON = false;
        intakeReversed = false;
    }

    // Call this to toggle the intake
    public void toggleFirstIntake() {
        if (intakeON) {
            stopFirstIntake();
        } else {
            startFirstIntake();
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("FirstIntakeOn", intakeON);
        SmartDashboard.putBoolean("FirstIntakeReversed", intakeReversed);
        SmartDashboard.putNumber("Intake Encoder", m_encoder.getPosition());
    }
}