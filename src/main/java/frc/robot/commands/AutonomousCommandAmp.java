// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class AutonomousCommandAmp extends SequentialCommandGroup {
  public AutonomousCommandAmp(SwerveDrive m_swerveDrive, Intake m_intake, Flywheel m_flywheel) {
    super(
      new InstantCommand(m_swerveDrive::resetPosition),
      new MoveX(m_swerveDrive, Units.inchesToMeters(4)),
      new MoveY(m_swerveDrive, Units.inchesToMeters(-17)),
      new ShootAmp(m_flywheel, m_intake)
    );
  }
}