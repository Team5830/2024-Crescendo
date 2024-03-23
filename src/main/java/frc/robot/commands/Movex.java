package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveTrain;
import frc.robot.subsystems.SwerveDrive;

public class MoveX extends PIDCommand {
  private SwerveDrive drive;
  private double targetDistance;

  public MoveX(SwerveDrive drive,double targetDistanceX) {
    super(
      new PIDController(DriveTrain.driveControllerKp*9, DriveTrain.driveControllerKi, DriveTrain.driveControllerKd*3),
      ()->-drive.getDriveOffset(),
      0,
      output -> drive.drive(-output, 0,0,false,true), drive);

    getController().setTolerance(DriveTrain.lAlignTolerance, DriveTrain.lMaxAlignSpeed);

    this.drive=drive;
    this.targetDistance=targetDistanceX;
  }

  @Override
  public void initialize() {
    super.initialize();

    drive.resetPosition();

    double setpointValue = -drive.getDriveOffset()+targetDistance;
    m_setpoint = ()->setpointValue;
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
