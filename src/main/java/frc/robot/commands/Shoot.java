package frc.robot.commands;

//import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Flywheel;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Shoot extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Flywheel m_flywheel;
  //private final Conveyor m_conveyor;

  public Shoot(Flywheel subsystemFLY) {
    m_flywheel = subsystemFLY;
   // m_conveyor = subsystemCO;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_flywheel.isshooteron) {
      m_flywheel.shooteroff();
    } else {
      m_flywheel.shooteron();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_flywheel.readyToShoot()) {
      // m_conveyor.conveyor2ON();
      //m_conveyor.conv2up();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_conveyor.conveyor2OFF();
    m_flywheel.shooteroff();
  }

  // Returns true when the command should end.
  //@Override
  //public boolean isFinished() {
    //return (m_conveyor.atTarget());
  
}