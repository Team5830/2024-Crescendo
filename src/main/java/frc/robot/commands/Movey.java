package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveTrain;
import frc.robot.subsystems.SwerveDrive;

public class Movey extends PIDCommand {
  private SwerveDrive drive;
  private double targetDistance;

  public Movey(SwerveDrive drive,double targetDistanceY) {
    super(
      new PIDController(DriveTrain.driveControllerKp*9, DriveTrain.driveControllerKi, DriveTrain.driveControllerKd*3), 
      ()->-drive.getDriveOffset(),
      0,
      output -> drive.drive(0, -output,0,false,true), drive);

    getController().setTolerance(DriveTrain.lAlignTolerance, DriveTrain.lMaxAlignSpeed);

    this.drive=drive;
    this.targetDistance=targetDistanceY;
  }

  
  @Override
  public void initialize() {
    super.initialize();

    drive.resetPosition();

    double setpointValue = drive.getDriveOffset()+targetDistance;
    m_setpoint = ()->setpointValue;
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
