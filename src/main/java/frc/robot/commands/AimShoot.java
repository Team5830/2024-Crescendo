// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;
import frc.robot.utils.VisionResult;

public class AimShoot extends Command {
  /** Creates a new AimShoot. */
  private Flywheel m_flywheel;
  private Intake m_intake;
  private SwerveDrive m_drive;
  private Vision m_vision;
  private int targetTag;
  private PhotonTrackedTarget m_photonTarget;
  private List<PhotonTrackedTarget> targetList;
  private boolean finished; 
  private double range,yaw;
  private PIDController forwardController;
  private PIDController turnController;
  private VisionResult vresult;
  public AimShoot(Flywheel subsystemFly,Intake subsystemIntake, SwerveDrive subsystemDrive, Vision subsystemVision) {
    m_flywheel = subsystemFly;
    m_intake = subsystemIntake;
    m_flywheel = subsystemFly;
    m_intake = subsystemIntake;
    m_vision = subsystemVision;
    finished = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_flywheel,m_intake,m_flywheel,m_intake, m_vision);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Get Speaker Target,  depending on side
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
            targetTag = 3;//4
        }
        if (ally.get() == Alliance.Blue) {
            targetTag = 1;//2
        }
    }
    else {
        targetTag = 0;
    }
    forwardController = new PIDController(Constants.vision.linearP, Constants.vision.linearI,
                    Constants.vision.linearD);
    turnController = new PIDController(Constants.vision.angularP, Constants.vision.angularI,
            Constants.vision.angularD);
  }

  public VisionResult calculateTargetMovement() {
    targetList = m_vision.getAprilTagVisionResult(targetTag);
      if (targetList.isEmpty()){
        DriverStation.reportError("Target: " + targetTag+" not in view", false);
        return new VisionResult(0,0);
      }
      m_photonTarget = targetList.get(0);
      range = m_vision.getAprilTagRange(targetTag);
      yaw = m_vision.getAprilTagYaw(targetTag);
      if (range < 5 && yaw < 10){
          DriverStation.reportWarning("Range to "+targetTag+" "+range+" meters Angle "+yaw, false);
          return new VisionResult(forwardController.calculate(range, Constants.vision.goalRangeMeters), -turnController.calculate(yaw, 0));
      }else{
        DriverStation.reportWarning("Tag is out of range", null);
      }
      return new VisionResult(0,0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    vresult = calculateTargetMovement();
    m_drive.drive() .vresult.forwardSpeed,vresult.rotationSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (range < 5 && yaw < )
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
