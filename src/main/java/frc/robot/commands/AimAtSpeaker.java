// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;
import frc.robot.utils.Utils;

public class AimAtSpeaker extends Command {
  private Flywheel m_flywheel;
  private Intake m_intake;
  private SwerveDrive m_drive;
  private Vision m_vision;
  private Arm m_arm;
  private int targetTag;
  private boolean finished;
  private double range, yaw;
  private PIDController forwardController;
  private PIDController turnController;
  private Translation2d translationToTarget;
  private Rotation2d rotationToTarget;

  public AimAtSpeaker(Flywheel subsystemFly, Intake subsystemIntake, SwerveDrive subsystemDrive,
      Vision subsystemVision, Arm arm) {
    m_flywheel = subsystemFly;
    m_intake = subsystemIntake;
    m_drive = subsystemDrive;
    m_vision = subsystemVision;
    m_arm = arm;
    finished = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_flywheel, m_intake, m_drive, m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Get Speaker Target, depending on side
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        targetTag = 4;// 3
        DriverStation.reportWarning("Red Alliance: Speaker " + targetTag, false);
      }
      if (ally.get() == Alliance.Blue) {
        targetTag = 4;// 1;//2
        DriverStation.reportWarning("Blue Alliance: Speaker " + targetTag, false);
      }
    } else {
      targetTag = 0;
    }
    finished = false;
    targetTag=2;

    forwardController = new PIDController(Constants.vision.linearP, Constants.vision.linearI,
        Constants.vision.linearD);
    turnController = new PIDController(Constants.vision.angularP, Constants.vision.angularI,
        Constants.vision.angularD);
  }

  public void DriveToTag() {
    Transform3d tag3dTranslation;
    double mag;
    m_vision.getAprilTagVisionResult(targetTag);
    if (m_vision.matched.isEmpty()) {
      DriverStation.reportError("Target: " + targetTag + " not in view", false);
      // translationToTarget = new Translation2d(0,0);
      // rotationToTarget = new Rotation2d(0);
      finished = true;
    }
    if (!finished) {
      range = m_vision.getAprilTagRange();
      yaw = m_vision.getAprilTagYaw()+5;
      boolean inRange = range < Constants.vision.speakerAimFarRange;
      // tag3dTranslation = m_vision.getAprilTagTransform();
      // translationToTarget = new
      // Translation2d(tag3dTranslation.getX(),tag3dTranslation.getY()); //Ignore Z
      // translation
      // Rotation2d targetYaw = PhotonUtils.getYawToPose( m_drive.getPose()
      // ,m_vision.getAprilTagPose(targetTag).toPose2d()); // This doesn't use the
      // vision measurement
              DriverStation.reportWarning("Range to " + targetTag + " " + range + " meters Angle " + yaw, false);
        // Scale X,Y proportionally
        mag = inRange ? 0 : /*forwardController.calculate(range, Constants.vision.speakerAimFarRange)*/1;

        m_drive.drive(mag * Math.cos(yaw)   *0   , mag /* * Math.sin(yaw)*/, turnController.calculate(Units.degreesToRadians(-yaw), 0),
            false,false);

      if (inRange) {
        double armAngle = ((range - Constants.vision.speakerAimCloseRange) / (Constants.vision.speakerAimFarRange - Constants.vision.speakerAimCloseRange))
                        * (Constants.vision.speakerAimFarAngle - Constants.vision.speakerAimCloseAngle) + Constants.vision.speakerAimCloseAngle;
        m_arm.move(armAngle);
      } else {
        DriverStation.reportWarning("Tag is out of range", false);
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveToTag();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Choose target specs... range, angle that is OK to shoot from
    if (m_arm.atTarget() && Math.abs(yaw) < 2) {
      finished = true;
    }
    finished = false;
    return finished;
  }
}
