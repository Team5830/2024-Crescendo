package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveTrain;
import frc.robot.subsystems.SwerveDrive;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class Movex extends PIDCommand {
  public Movex(SwerveDrive drive,double targetDistanceX) {
    super(new PIDController(DriveTrain.driveControllerKp, DriveTrain.driveControllerKi, DriveTrain.driveControllerKd),
        drive::getdriveoffset, drive.getdriveoffset()+targetDistanceX, output -> drive.drive(output, 0,0,false,0), drive);

    getController()
        .setTolerance(DriveTrain.lAlignTolerance, DriveTrain.lMaxAlignSpeed);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}
