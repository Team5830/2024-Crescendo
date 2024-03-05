// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.function.DoubleSupplier;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class AutonomousCommandC extends SequentialCommandGroup {
  /** Example static factory for an autonomous command. */

  public AutonomousCommandC(Flywheel m_flywheel, SwerveDrive m_drivetrain, Intake m_intake, Arm m_arm ) {
    new SequentialCommandGroup(
     new MoveArm(m_arm, -50).withTimeout(2),
     new Shoot(m_flywheel, m_intake),
     new WaitCommand(0.75),
     new Movex(m_drivetrain, -1)
    );
    
  }
}
