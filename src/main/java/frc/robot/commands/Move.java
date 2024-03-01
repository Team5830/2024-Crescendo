package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class Move extends PIDCommand {
  public Move(SwerveDrive drive,double targetDistanceMeters) {
    super(new PIDController(Constants.moveCommand.lP, Constants.moveCommand.lI, Constants.moveCommand.lD),
        drive::getDistance, drive.getDistance()+targetDistanceMeters, output -> drive.drive(output, output,0,false,0), drive);

    getController()
        .setTolerance(Constants.moveCommand.lAlignTolerance, Constants.moveCommand.lMaxAlignSpeed);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}
