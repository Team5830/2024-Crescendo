// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class AutonomousCommandSpeaker extends SequentialCommandGroup {
  public AutonomousCommandSpeaker(SwerveDrive m_swerveDrive, Intake m_intake, Flywheel m_flywheel, Arm m_arm) {
    super(
        new MoveArm(m_arm, Constants.arm.positionShoot),
        new WaitCommand(2),
        new Shoot(m_flywheel, m_intake),
        new MoveY(m_swerveDrive, -1)
    );
  }
}
