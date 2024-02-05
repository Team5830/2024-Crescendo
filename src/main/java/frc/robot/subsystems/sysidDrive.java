// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveTrain;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;

public class sysidDrive extends SubsystemBase {
  // The motors on the left side of the drive.
  private final CANSparkMax m_leftMotor = new CANSparkMax(DriveTrain.frontLeftDriveChannel, CANSparkMax.MotorType.kBrushless);

  private final CANSparkMax m_leftMotor2 = new CANSparkMax(DriveTrain.backLeftDriveChannel, CANSparkMax.MotorType.kBrushless);

  // The motors on the right side of the drive.
  private final CANSparkMax m_rightMotor = new CANSparkMax(DriveTrain.frontRightDriveChannel, CANSparkMax.MotorType.kBrushless);

  private final CANSparkMax m_rightMotor2 = new CANSparkMax(DriveTrain.backRightDriveChannel, CANSparkMax.MotorType.kBrushless);

  // The robot's drive
  private final DifferentialDrive m_drive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

  // The left-side drive encoder
  private final RelativeEncoder m_leftEncoder = m_leftMotor.getEncoder();

  // The right-side drive encoder
  private final RelativeEncoder m_rightEncoder = m_rightMotor.getEncoder();

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  // Create a new SysId routine for characterizing the drive.
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                m_leftMotor.setVoltage(volts.in(Volts));
                m_rightMotor.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_leftMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(m_leftEncoder.getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(m_leftEncoder.getVelocity(), MetersPerSecond));
                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_rightMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(m_rightEncoder.getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(m_rightEncoder.getVelocity(), MetersPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));

  /** Creates a new Drive subsystem. */
  public sysidDrive() {
    // Add the second motors on each side of the drivetrain
    m_leftMotor2.follow(m_leftMotor);
    m_rightMotor2.follow(m_rightMotor);
    m_leftEncoder.setPositionConversionFactor(12.5 * 2.54 / 6.55 / 100);
    m_rightEncoder.setPositionConversionFactor(12.5 * 2.54 / 6.55 / 100);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    // Sets the distance per pulse for the encoders
    //m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    //m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
    
  }

  /**
   * Returns a command that drives the robot with arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public Command arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot) {
    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    return run(() -> m_drive.arcadeDrive(fwd.getAsDouble(), rot.getAsDouble()))
        .withName("arcadeDrive");
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
