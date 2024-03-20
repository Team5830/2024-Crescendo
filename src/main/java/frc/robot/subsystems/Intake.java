// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.filter.Debouncer;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
public class Intake extends SubsystemBase {
    public boolean intakeON = false;
    public boolean intakeReversed = false;
    public DigitalInput noteSensor;
    public Debouncer m_debouncer; 
    CANSparkMax m_motorBottom;
     CANSparkMax m_motorTop;
    SparkPIDController m_pidController;

    RelativeEncoder m_encoder;

    public Intake() {
        try {
            m_motorBottom = new CANSparkMax(Constants.intake.motorChannel, MotorType.kBrushless);
             m_motorTop = new CANSparkMax(Constants.intake.motorChannelTop, MotorType.kBrushless);
            noteSensor = new DigitalInput(0);
            m_debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
           m_motorBottom.restoreFactoryDefaults();
        m_motorTop.restoreFactoryDefaults();
        m_motorTop.follow(m_motorBottom, true);    

    m_pidController = m_motorBottom.getPIDController();
    m_encoder = m_motorBottom.getEncoder();
    m_encoder.setPositionConversionFactor(3.141592652);
    m_pidController.setP(Constants.intake.kP);
    m_pidController.setI(Constants.intake.kI);
    m_pidController.setD(Constants.intake.kD);
    m_pidController.setFF(Constants.intake.kFF);
    m_pidController.setOutputRange(Constants.intake.kMinOutput, Constants.intake.kMaxOutput);}
    catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating intake: " + ex.getMessage(), true);
        
        }
    }
    //     m_pidController = m_motor.getPIDController();
    //      set PID coefficients
    //     m_pidController.setP(Constants.Intake.P);
    //     m_pidController.setI(Constants.Intake.I);
    //     m_pidController.setD(Constants.Intake.D);
    //     m_pidController.setIZone(Constants.Intake.zI);
    //     m_pidController.setFF(Constants.Intake.F);
    //     m_pidController.setOutputRange(Constants.Intake.kMinOutput, Constants.Intake.kMaxOutput);
    // }

    public void stopIntake() {
        //m_motorTop.set(0);
        m_motorBottom.set(0);
    }

    public void startFirstIntake() {
        m_motorTop.set(-Constants.intake.firstIntakeBottomSpeed);
        m_motorBottom.set(Constants.intake.firstIntakeTopSpeed);
        intakeON = true;
    }

    public void reverseFirstIntake() {
        m_motorBottom.set(-Constants.intake.firstIntakeBottomSpeed*1.5);
        m_motorTop.set(-Constants.intake.firstIntakeTopSpeed);
        intakeON = true;
        intakeReversed = true;
    }

    public void stopFirstIntake() {
        m_motorBottom.set(0);
         m_motorTop.set(0);
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
        SmartDashboard.putNumber("intake encoder position", m_encoder.getPosition());
        SmartDashboard.putNumber("Intake Encoder", m_encoder.getPosition());
    }

    public boolean noteSensorIsDetected(){
       return m_debouncer.calculate(!noteSensor.get());
    }
    public boolean noteSensorIsNotDetected(){
       return m_debouncer.calculate(noteSensor.get());
    }
public void PIDOFF()
{
    m_pidController.setReference(0, ControlType.kDutyCycle);
}

public void reverseIntake(){
    System.out.println("reverseIntake");
    System.out.println(m_encoder.getPosition() -2*Math.PI);
    m_pidController.setReference(m_encoder.getPosition() -2*Math.PI, ControlType.kPosition);
    }   
}