package frc.robot.commands;

import java.util.Optional;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

public class TestTrajectory extends Command {
  private SwerveDrive m_drive;
  private PIDController forwardController;
  private PIDController turnController;
  private Translation2d translationToTarget;
  private Rotation2d rotationToTarget;
  private SequentialCommandGroup finalCommand;
  private Pose2d finalPose;

  public TestTrajectory(SwerveDrive subsystemDrive, Vision subsystemVision) {
    m_drive = subsystemDrive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements( m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    forwardController = new PIDController(Constants.vision.linearP, Constants.vision.linearI,
        Constants.vision.linearD);
    turnController = new PIDController(Constants.vision.angularP, Constants.vision.angularI,
        Constants.vision.angularD);
    //translationToTarget = new Translation2d(0, 1);
    //rotationToTarget = new Rotation2d(180);
    // Report values for checking
    //SmartDashboard.putNumber("Translation To Target X", translationToTarget.getX());
    //SmartDashboard.putNumber("Translation To Target Y", translationToTarget.getY());
    //SmartDashboard.putNumber("Rotation To Target ", rotationToTarget.getDegrees());
    // Use translationToTarget and rotationToTarget to create final pose and
    // intermediate points for trajectory
    finalPose = new Pose2d(2, 0, Rotation2d.fromDegrees(180));
    List<Translation2d> IntermediatePoints = List.of(
        new Translation2d(0.5, 0),
        new Translation2d(1, 0),
        new Translation2d(1.5, 0));
    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        Constants.DriveTrain.maxSpeed,
        Constants.DriveTrain.maxAcceleration)
        .setKinematics(m_drive.m_kinematics);

    // 2. Generate trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        IntermediatePoints,
        finalPose,
        trajectoryConfig);

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(Constants.DriveTrain.driveControllerKp,
        Constants.DriveTrain.driveControllerKi, Constants.DriveTrain.driveControllerKd);
    PIDController yController = new PIDController(Constants.DriveTrain.driveControllerKp,
        Constants.DriveTrain.driveControllerKi, Constants.DriveTrain.driveControllerKd);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        Constants.TurnPID.P, Constants.TurnPID.I, Constants.TurnPID.D,
        new TrapezoidProfile.Constraints(
            Constants.DriveTrain.maxAngularVelocity,
            Constants.DriveTrain.maxAngularAcceleration));
    // Change to degrees?
    thetaController.enableContinuousInput(-180, 179);

    // 4. Construct command to follow trajectory
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        m_drive::getPose,
        m_drive.m_kinematics,
        xController,
        yController,
        thetaController,
        m_drive::setModuleStates,
        m_drive);

    finalCommand = new SequentialCommandGroup(
        new InstantCommand(() -> m_drive.resetOdometry(trajectory.getInitialPose())),
        swerveControllerCommand,
        new InstantCommand(() -> m_drive.stopModules()));
    // Run it
    finalCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  //Don't override isFinished
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ( m_drive.getPose( )== finalPose ){
        return true;
    }else{
        return false;
    }
    }
}