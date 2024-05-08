package frc.robot.commands;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.utils.SequentialCommandGroupMod;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public final class ShootAmp extends SequentialCommandGroupMod {
 Flywheel subsystemFLY;
 Intake intake;

  public ShootAmp(Flywheel subsystemFLY, Intake intake) {
    super(
      new InstantCommand(subsystemFLY::shooterHalf),
      new WaitCommand(1),
      new InstantCommand(intake::startFirstIntake),
      new WaitUntilCommand(intake::noteSensorIsNotDetected),
      new WaitCommand(0.5)
      // Run end commands at this point
    );

    addRequirements(subsystemFLY, intake);

    this.subsystemFLY=subsystemFLY;
    this.intake=intake;
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);

    intake.stopFirstIntake();
    subsystemFLY.shooterOff();
  }
}
