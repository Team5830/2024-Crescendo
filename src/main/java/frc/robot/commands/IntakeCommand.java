package frc.robot.commands;
import frc.robot.subsystems.Intake;
import frc.robot.utils.SequentialCommandGroupMod;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public final class IntakeCommand extends SequentialCommandGroupMod {
 Intake m_intake;

  public IntakeCommand( Intake m_intake) {
    super(
        new InstantCommand(m_intake::startFirstIntake),
        new WaitUntilCommand(m_intake::noteSensorIsDetected).withTimeout(10),
        new InstantCommand(m_intake::reverseFirstIntake),
        new WaitCommand(0.06)
      // Run end commands at this point
    );

    addRequirements(m_intake);

    this.m_intake=m_intake;
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);

    m_intake.stopFirstIntake();
  }
}
