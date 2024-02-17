// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Represents a swerve drive style drivetrain. */
public class SwerveDrive extends SubsystemBase {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.438, 0.438);
  private final Translation2d m_frontRightLocation = new Translation2d(0.438, -0.438);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.438, 0.438);
  private final Translation2d m_backRightLocation = new Translation2d(-0.438, -0.438);
  private final AHRS ahrs = new AHRS(SPI.Port.kMXP);
      //odometry = new DifferentialDriveOdometry(ahrs.getRotation2d(), m_frontLeftLocation.);
    
    //ahrs.reset();
  private final SwerveModule m_frontLeft = new SwerveModule(
      Constants.DriveTrain.frontLeftDriveChannel,
      Constants.DriveTrain.frontLeftTurnChannel,
      true);
  private final SwerveModule m_frontRight = new SwerveModule(
      Constants.DriveTrain.frontRightDriveChannel,
      Constants.DriveTrain.frontRightTurnChannel,
      true);
  private final SwerveModule m_backLeft = new SwerveModule(
      Constants.DriveTrain.backLeftDriveChannel,
      Constants.DriveTrain.backLeftTurnChannel,
      true);
  private final SwerveModule m_backRight = new SwerveModule(
      Constants.DriveTrain.backRightDriveChannel,
      Constants.DriveTrain.backRightTurnChannel,
      true);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  /* Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings. The numbers used
  below are robot specific, and should be tuned. */
  private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          m_kinematics,
          Constants.DriveTrain.invertNavX ? ahrs.getRotation2d().unaryMinus() : ahrs.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          },
          new Pose2d(),
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, m_poseEstimator.getEstimatedPosition().getRotation())
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
            periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.DriveTrain.maxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_poseEstimator.update(
        Constants.DriveTrain.invertNavX ? ahrs.getRotation2d().unaryMinus() : ahrs.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });

    // Also apply vision measurements. We use 0.3 seconds in the past as an example
    // -- on a real robot, this must be calculated based either on latency or
    // timestamps.
    m_poseEstimator.addVisionMeasurement(
        ExampleGlobalMeasurementSensor.getEstimatedGlobalPose(
            m_poseEstimator.getEstimatedPosition()),
        Timer.getFPGATimestamp() - 0.3);
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("DriveP", Constants.DriveTrain.driveControllerKp);
    SmartDashboard.putNumber("DriveI", Constants.DriveTrain.driveControllerKi);
    SmartDashboard.putNumber("DriveD", Constants.DriveTrain.driveControllerKd);
    SmartDashboard.putNumber("TurnP", Constants.DriveTrain.turnControllerKp);
    SmartDashboard.putNumber("TurnI", Constants.DriveTrain.turnControllerKi);
    SmartDashboard.putNumber("DriveD", Constants.DriveTrain.turnControllerKd);
    SmartDashboard.putNumber("FrontLeft Angle", m_frontLeft.Angle());
    SmartDashboard.putNumber("FrontLeft Position", m_frontLeft.Offset());
    SmartDashboard.putNumber("FrontRight Angle", m_frontRight.Angle());
    SmartDashboard.putNumber("FrontRight Position", m_frontRight.Offset());
    SmartDashboard.putNumber("BackLeft Angle", m_backLeft.Angle());
    SmartDashboard.putNumber("BackLeft Position", m_backLeft.Offset());
    SmartDashboard.putNumber("BackRight Angle", m_backRight.Angle());
    SmartDashboard.putNumber("BackRight Position", m_backRight.Offset());
    SmartDashboard.putNumber("NAVX Heading", Constants.DriveTrain.invertNavX ? -ahrs.getAngle() : ahrs.getAngle());
  }
}
