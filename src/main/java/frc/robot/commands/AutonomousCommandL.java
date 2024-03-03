// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

/*import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.function.DoubleSupplier;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;*/
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class AutonomousCommandL extends SequentialCommandGroup {
  /** Example static factory for an autonomous command. */

  public AutonomousCommandL(/*Flywheel m_flywheel, Drivetrain m_drivetrain, Intake m_intake, Arm m_arm*/) {
    addCommands(
     /*new Shoot(m_flywheel),
     wait(.75),
     new Move(-.5, m_drivtrain),
     new Turn(45 degrees right, m_drivetrain), 
     new Move(-3.0, m_drivetrain),
     new Postioning(m_arm, m_intake, Position2.armangle, Start.m_intake),
     new Wait(.75),
     new Move(-1),
     new Positioning(m_arm, Position1.armangle)
     new Stop(m_intake, m_flywheel, m_drivetrain)
     */ 
    );
  }
}