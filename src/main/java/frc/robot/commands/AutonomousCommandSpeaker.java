// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class AutonomousCommandSpeaker extends SequentialCommandGroup {
  public AutonomousCommandSpeaker(SwerveDrive m_swerveDrive, Intake m_intake, Flywheel m_flywheel, Arm m_arm) {
    super(
        new InstantCommand(m_swerveDrive::resetPosition),
        new MoveArm(m_arm, -52),
        new WaitCommand(2),
        new Shoot(m_flywheel, m_intake),
        new WaitCommand(2),
        new MoveArm(m_arm, -94),
        new WaitCommand(.5),
        new MoveY(m_swerveDrive, -2)
    
        // new ParallelCommandGroup(new MoveY(m_swerveDrive, -2), new IntakeCommand(m_intake)),
        // new MoveArm(m_arm, -52),
        // new InstantCommand(m_swerveDrive::resetPosition)
    );
  }
}
