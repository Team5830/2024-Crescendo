package frc.robot.commands;
import frc.robot.subsystems.Flywheel;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Shoot extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Flywheel m_flywheel;
  private boolean turnedON = false;

  public Shoot(Flywheel subsystemFLY) {
    m_flywheel = subsystemFLY;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_flywheel.shooterOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_flywheel.readyToShoot()) {
      turnedON = true;
    }
    // if (turnedON & m_conveyor.ballsensor2.get()) {
    // m_conveyor.conveyor2OFF();
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_flywheel.shooterOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (turnedON) {
      return true;
    } else {
      return false;
    }
  }
}