// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.ExternalFollower;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
//import frc.robot.commands.Flywheel_test;

public class Flywheel extends SubsystemBase {
   CANSparkMax m_leftlead;
   CANSparkMax m_rightfollow;
   public SparkPIDController m_pidController;
   RelativeEncoder m_encoder;
   public boolean isshooteron = false;

   public double maxRPM, maxVel, minVel, maxAcc, allowedErr;
   public double motorspeed;

   public class PidVals {
      public double kP = FlywheelC.kP;
      public double kI = FlywheelC.kI;
      public double kD = FlywheelC.kD;
      public double kIz = FlywheelC.kIz;
      public double kFF = FlywheelC.kFF;
      public double kMaxOutput = FlywheelC.kMaxOutput;
      public double kMinOutput = FlywheelC.kMinOutput;
      public double motorspeed = FlywheelC.shootermotorspeed;
      public double speedTolerance = FlywheelC.speedTolerance;
   }

   PidVals oldPidVals = new PidVals();
   public PidVals pidVals = new PidVals();
   ShuffleboardTab FlywheelControl = Shuffleboard.getTab("Flywheel");

   public Flywheel() {
      try {
         // m_rightfollow = new CANSparkMax(CANBusID.kRightFlywheel,
         // MotorType.kBrushless);
         m_leftlead = new CANSparkMax(CANBusID.kLeftFlywheel, MotorType.kBrushless);
         m_leftlead.restoreFactoryDefaults();
         // m_rightfollow.restoreFactoryDefaults();
         m_leftlead.follow(ExternalFollower.kFollowerDisabled, 0);
         // m_rightfollow.follow(m_leftlead, true);

         m_pidController = m_leftlead.getPIDController();
         m_encoder = m_leftlead.getEncoder();
      } catch (RuntimeException ex) {
         DriverStation.reportError("error loading failed" + ex.getMessage(), true);
      }
      // Put PID coefficients on Dashboard

      SmartDashboard.putNumber("Flywheel P", pidVals.kP);
      SmartDashboard.putNumber("Flywheel I", pidVals.kI);
      SmartDashboard.putNumber("Flywheel D", pidVals.kD);
      SmartDashboard.putNumber("Flywheel kIz", pidVals.kIz);
      SmartDashboard.putNumber("Flywheel F", pidVals.kFF);
      SmartDashboard.putNumber("Flywheel MaxOutput", pidVals.kMaxOutput);
      SmartDashboard.putNumber("Flywheel MinOutput", pidVals.kMinOutput);
      SmartDashboard.putNumber("Flywheel motorspeed", pidVals.motorspeed);

       //FlywheelControl.add("Flywheel Control", m_pidController);
      // FlywheelControl.add("Flywheel", m_flywheel);
      // FlywheelControl.add("Flywheel", new Flywheel_test(m_flywheel));

      FlywheelControl.add("Flywheel P", pidVals.kP);
      FlywheelControl.add("Flywheel I", pidVals.kI);
      FlywheelControl.add("Flywheel D", pidVals.kD);
      FlywheelControl.add("Flywheel kIz", pidVals.kIz);
      FlywheelControl.add("Flywheel F", pidVals.kFF);
      FlywheelControl.add("Flywheel MaxOutput", pidVals.kMaxOutput);
      FlywheelControl.add("Flywheel MinOutput", pidVals.kMinOutput);
      FlywheelControl.add("Flywheel motorspeed", pidVals.motorspeed);
      FlywheelControl.add("Flywheel Speed Test", m_encoder.getVelocity());
      FlywheelControl.add("Flywheel On", isshooteron);

      updatePIDValues();
   }

   @Override
   public void periodic() {
      /*
       * try {
       * // SmartDashboard.putBoolean("Flywheel On", isshooteron);
       * 
       * } catch (RuntimeException ex) {
       * DriverStation.reportError("Shooter: Not able to get velocity " +
       * ex.getMessage(), true);
       * }
       */
      SmartDashboard.putNumber("Flywheel Speed ", m_encoder.getVelocity());
      // SmartDashboard.putNumber("Flywheel Current ",
      // m_rightfollow.getOutputCurrent());
      // SmartDashboard.putNumber("Flywheel Current ",
      // m_rightfollow.getVoltageCompensationNominalVoltage());

   }

   // Load PID coefficients from Dashboard
   public void updatePIDValues() {

      pidVals.kP = SmartDashboard.getNumber("Flywheel P", FlywheelC.kP);
      pidVals.kI = SmartDashboard.getNumber("Flywheel I", FlywheelC.kI);
      pidVals.kD = SmartDashboard.getNumber("Flywheel D", FlywheelC.kD);
      pidVals.kIz = SmartDashboard.getNumber("Flywheel kIz", FlywheelC.kIz);
      pidVals.kFF = SmartDashboard.getNumber("Flywheel F", FlywheelC.kFF);
      pidVals.kMaxOutput = SmartDashboard.getNumber("Flywheel MaxOutput",
            FlywheelC.kMaxOutput);
      pidVals.kMinOutput = SmartDashboard.getNumber("Flywheel MinOutput",
            FlywheelC.kMinOutput);
      motorspeed = SmartDashboard.getNumber("Flywheel motorspeed", pidVals.motorspeed);

      m_pidController.setP(pidVals.kP);
      m_pidController.setI(pidVals.kI);
      m_pidController.setD(pidVals.kD);
      m_pidController.setIZone(pidVals.kIz);
      m_pidController.setFF(pidVals.kFF);
      m_pidController.setOutputRange(pidVals.kMinOutput, pidVals.kMaxOutput);
   }

   public boolean getShooterState() {
      return isshooteron;
   }

   public void shooteron() {
      m_pidController.setReference(motorspeed, ControlType.kVelocity);
      isshooteron = true;
   }

   public void shooterGo() {
      m_leftlead.set(0.1);// may be lowered or raised again
      isshooteron = true;
   }

   public void shooteroff() {
      m_leftlead.set(0);
      isshooteron = false;
   }

   public boolean readyToShoot() {
      return (Math.abs(motorspeed - m_encoder.getVelocity()) < pidVals.speedTolerance);
   }
}