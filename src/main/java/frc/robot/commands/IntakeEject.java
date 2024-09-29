package frc.robot.commands;
import frc.robot.subsystems.Intake;
import frc.robot.utils.SequentialCommandGroupMod;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class IntakeEject extends SequentialCommandGroupMod {
 Intake m_intake;

  public IntakeEject( Intake m_intake) {
    super(
        new InstantCommand(m_intake::reverseFirstIntake),
        new WaitCommand(1)
        // Run end commands at this point
    );

    this.m_intake=m_intake;
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);

    m_intake.stopFirstIntake();
  }
}
