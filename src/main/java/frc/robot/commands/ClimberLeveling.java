package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.SwerveDrive;

public class ClimberLeveling extends PIDCommand {
  public ClimberLeveling(Climber climber, SwerveDrive swerveDrive) {
    super(
        new PIDController(Constants.TurnPID.P, Constants.TurnPID.I, Constants.TurnPID.D),
        swerveDrive::getRoll,
        // Set reference to target
        Constants.climberLeveling.targetValue,
        // Pipe output to turn robot
        output -> climber.changeBoth(output),
        // Require the drive
        climber);
    getController()
        .setTolerance(Constants.climberLeveling.positionTolerance, Constants.climberLeveling.velocityTolerance);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}