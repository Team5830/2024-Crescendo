package frc.robot.commands;

import frc.robot.subsystems.Flywheel;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Flywheel_test extends CommandBase {
  private final Flywheel m_flywheel;

  public Flywheel_test(Flywheel subsystemFLY) {
    m_flywheel = subsystemFLY;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Get values from dashboard if not available use values from constants
    m_flywheel.updatePIDValues();
    m_flywheel.shooteron();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_flywheel.shooteroff();
  }

  // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  // if (m_flywheel.readyToShoot()){
  // return true;
  // } else{
  // return false;
  // }
  // }
}