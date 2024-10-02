// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class AutonomousCommandSpeaker extends SequentialCommandGroup {
  public AutonomousCommandSpeaker(SwerveDriveSub m_swerveDrive, Intake m_intake, Flywheel m_flywheel, Arm m_arm) {
    super(
      new MoveArm(m_arm, -52 ),
      new WaitCommand(2),
      new Shoot(m_flywheel, m_intake),
      new WaitCommand(2),
      new MoveArm(m_arm, -94),
      new WaitCommand(0.5)
      //m_swerveDrive.d
      //new InstantCommand( m_swerveDrive.drive(new Translation2d(-2,0) ,0.0,false) 
      );
  }
}