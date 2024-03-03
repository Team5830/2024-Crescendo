// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.function.DoubleSupplier;

import com.fasterxml.jackson.databind.introspect.ConcreteBeanPropertyBase;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class AutonomousCommandA extends SequentialCommandGroup {
  /** Example static factory for an autonomous command. */

  public AutonomousCommandA(Flywheel m_flywheel, SwerveDrive m_drivetrain, Intake m_intake, Arm m_arm, DoubleSupplier periodSeconds ) {
    addCommands(
    new Move(m_drivetrain, 1),
    new Shoot(m_flywheel,m_intake),
    new WaitCommand(0.2),
    new MoveArm(m_arm, -94) //Move arm for pickup
    );
  }
}