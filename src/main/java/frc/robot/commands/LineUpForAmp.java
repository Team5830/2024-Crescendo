package frc.robot.commands;

import java.util.Optional;
import java.util.List;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

public class LineUpForAmp extends Command {
  private Flywheel m_flywheel;
  private Intake m_intake;
  private SwerveDrive m_drive;
  private Vision m_vision;
  private int targetTag;
  private boolean finished;
  private double range, yaw;
  private PIDController forwardController;
  private PIDController turnController;
  private Translation2d translationToTarget;
  private Rotation2d rotationToTarget;
  private SequentialCommandGroup finalCommand;

  public LineUpForAmp(Flywheel subsystemFly, Intake subsystemIntake, SwerveDrive subsystemDrive,
      Vision subsystemVision) {
    m_flywheel = subsystemFly;
    m_intake = subsystemIntake;
    m_drive = subsystemDrive;
    m_intake = subsystemIntake;
    m_vision = subsystemVision;
    finished = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_flywheel, m_intake, m_drive, m_intake, m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Get Speaker Target, depending on side
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        targetTag = 5;
      }
      if (ally.get() == Alliance.Blue) {
        targetTag = 6;
      }
    } else {
      targetTag = 0;
    }
    forwardController = new PIDController(Constants.vision.linearP, Constants.vision.linearI,
        Constants.vision.linearD);
    turnController = new PIDController(Constants.vision.angularP, Constants.vision.angularI,
        Constants.vision.angularD);
    //FindTag(); // sets translation and rotation to target
    // Report values for checking
    //SmartDashboard.putNumber("Translation To Target X", translationToTarget.getX());
    //SmartDashboard.putNumber("Translation To Target Y", translationToTarget.getY());
    //SmartDashboard.putNumber("Rotation To Target ", rotationToTarget.getDegrees());
    // Use translationToTarget and rotationToTarget to create final pose and
    // intermediate points for trajectory
    Pose2d finalPose = new Pose2d( 2,0, Rotation2d.fromDegrees(180));
    List<Translation2d> IntermediatePoints = List.of(
        new Translation2d(0.5,0),
       new Translation2d(1.5,0)
       //      new Translation2d(1,0)
        );
    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        Constants.DriveTrain.maxSpeed,
        Constants.DriveTrain.maxAcceleration)
        .setKinematics(m_drive.m_kinematics);

    // 2. Generate trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        IntermediatePoints,
        finalPose,
        trajectoryConfig);

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(Constants.DriveTrain.driveControllerKp,
        Constants.DriveTrain.driveControllerKi, Constants.DriveTrain.driveControllerKd);
    PIDController yController = new PIDController(Constants.DriveTrain.driveControllerKp,
        Constants.DriveTrain.driveControllerKi, Constants.DriveTrain.driveControllerKd);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        Constants.TurnPID.P*10, Constants.TurnPID.I, Constants.TurnPID.D,
        new TrapezoidProfile.Constraints(
            Constants.DriveTrain.maxAngularVelocity,
            Constants.DriveTrain.maxAngularAcceleration));
    // Change to degrees?
    thetaController.enableContinuousInput(-179, 180);

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

 /*  public void FindTag() {
    Transform3d tag3dTranslation;
    double mag;
    m_vision.getAprilTagVisionResult(targetTag);
    if (m_vision.matched.isEmpty()) {
      DriverStation.reportWarning("Target: " + targetTag + " not in view", false);
      finished = true;
    }
     if (!finished) {
      range = m_vision.getAprilTagRange();
      yaw = m_vision.getAprilTagYaw();
      tag3dTranslation = m_vision.getAprilTagTransform();
      translationToTarget = new Translation2d(tag3dTranslation.getX(), tag3dTranslation.getY()); // Ignore Z translation
      rotationToTarget = new Rotation2d(yaw);
      // Rotation2d targetYaw = PhotonUtils.getYawToPose( m_drive.getPose()
      // ,m_vision.getAprilTagPose(targetTag).toPose2d()); // This doesn't use the
      // vision measurement
       if (range < 3 && yaw < 100) { // Only drive if Tag is within 3 meters and yaw is valid
        DriverStation.reportWarning("Range to " + targetTag + " " + range + " meters Angle " + yaw, false);
        // Scale X,Y proportionally
        // mag = forwardController.calculate(range, Constants.vision.goalRangeMeters);
        // m_drive.drive(mag*Math.cos(yaw),mag*Math.sin(yaw),
        // -turnController.calculate(yaw*(180.0/Math.PI), 0),false);
      } else {
        DriverStation.reportWarning("Tag is out of range", null);
        finished = true;
      }
    }
  }*/

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Choose target specs... range, angle that is OK to shoot from
    return finalCommand.isFinished();
  }
}
