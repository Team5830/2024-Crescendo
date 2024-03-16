package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

/** A command that will turn the robot to the specified angle. */
public class TurnCommand extends PIDCommand {
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  public TurnCommand(double targetAngleDegrees, SwerveDrive drive) {
    super(
        new PIDController(Constants.TurnPID.P, Constants.TurnPID.I, Constants.TurnPID.D),
        // Close loop on heading
        drive::getAngle,
        // Set reference to target
        targetAngleDegrees,
        // Pipe output to turn robot
        output -> drive.drive(0,0,output,false),
        // Require the drive
        drive);
    drive.resetHeading();
    System.out.print(String.format("Turn Target %f\n", targetAngleDegrees));
    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(Constants.TurnPID.Tolerance, Constants.TurnPID.TurnRateTolerance);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}