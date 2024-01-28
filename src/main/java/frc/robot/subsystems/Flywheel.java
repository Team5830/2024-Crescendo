package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase {
   CANSparkMax flyWheelController;
   public SparkPIDController m_pidController;
   RelativeEncoder m_encoder;
   public boolean isshooteron = false;

   public double maxRPM, maxVel, minVel, maxAcc, allowedErr;
   public double motorspeed;

   public class PidVals {
      public double kP = Constants.Flywheel.controllerKp;
      public double kI = Constants.Flywheel.controllerKi;
      public double kD = Constants.Flywheel.controllerKd;
      public double kIz = Constants.Flywheel.controllerkIz;
      public double kFF = Constants.Flywheel.controllerKf;
      public double kMaxOutput = Constants.Flywheel.MaxOutput;
      public double kMinOutput = Constants.Flywheel.MinOutput;
      public double motorspeed = Constants.Flywheel.shootermotorspeed;
      public double speedTolerance = Constants.Flywheel.Tolerance;
   }

   PidVals oldPidVals = new PidVals();
   public PidVals pidVals = new PidVals();
   ShuffleboardTab FlywheelControl = Shuffleboard.getTab("Flywheel");

   public Flywheel() {
      try {
         flyWheelController = new CANSparkMax(Constants.Flywheel.channel, MotorType.kBrushless);
         flyWheelController.restoreFactoryDefaults();
         m_pidController = flyWheelController.getPIDController();
         m_encoder = flyWheelController.getEncoder();
      } catch (RuntimeException ex) {
         DriverStation.reportError("error loading failed" + ex.getMessage(), true);
      }
      // Put PID coefficients on Dashboard

      SmartDashboard.putNumber("Flywheel P",    pidVals.kP);
      SmartDashboard.putNumber("Flywheel I",    pidVals.kI);
      SmartDashboard.putNumber("Flywheel D",    pidVals.kD);
      SmartDashboard.putNumber("Flywheel kIz",  pidVals.kIz);
      SmartDashboard.putNumber("Flywheel F",    pidVals.kFF);
      SmartDashboard.putNumber("Flywheel MaxOutput", pidVals.kMaxOutput);
      SmartDashboard.putNumber("Flywheel MinOutput", pidVals.kMinOutput);
      SmartDashboard.putNumber("Flywheel motorspeed", pidVals.motorspeed);

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
      
      SmartDashboard.putNumber("Flywheel Speed ", m_encoder.getVelocity());
   }

   // Load PID coefficients from Dashboard
   public void updatePIDValues() {

      pidVals.kP = SmartDashboard.getNumber("Flywheel P", Constants.Flywheel.controllerKp);
      pidVals.kI = SmartDashboard.getNumber("Flywheel I", Constants.Flywheel.controllerKi);
      pidVals.kD = SmartDashboard.getNumber("Flywheel D", Constants.Flywheel.controllerKd);
      pidVals.kIz = SmartDashboard.getNumber("Flywheel kIz", Constants.Flywheel.controllerkIz);
      pidVals.kFF = SmartDashboard.getNumber("Flywheel F", Constants.Flywheel.controllerKf);
      pidVals.kMaxOutput = SmartDashboard.getNumber("Flywheel MaxOutput",
            Constants.Flywheel.MaxOutput);
      pidVals.kMinOutput = SmartDashboard.getNumber("Flywheel MinOutput",
            Constants.Flywheel.MinOutput);
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
      flyWheelController.set(0.1);// may be lowered or raised again
      isshooteron = true;
   }

   public void shooteroff() {
      flyWheelController.set(0);
      isshooteron = false;
   }

   public boolean readyToShoot() {
      return (Math.abs(motorspeed - m_encoder.getVelocity()) < pidVals.speedTolerance);
   }
}