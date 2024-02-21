package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

public class Positioning extends SequentialCommandGroup {
    private Arm m_arm;

    public Positioning(Arm arm, double armAngle) {
        this.m_arm = arm;

        addCommands(new MoveArm(m_arm, armAngle));

    }
}