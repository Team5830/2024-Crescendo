package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class SmartDashboardCmd extends Command {
    public SmartDashboardCmd(SwerveDrive m_sdrive){
        SmartDashboard.putNumber("P Gain", PIDs.driveControllerKp);
        SmartDashboard.putNumber("I Gain", PIDs.driveControllerKi);
        SmartDashboard.putNumber("D Gain", PIDs.driveControllerKd);
        SmartDashboard.putNumber("P Gain", PIDs.turnControllerKd);
        SmartDashboard.putNumber("I Gain", PIDs.turnControllerKd);
        SmartDashboard.putNumber("D Gain", PIDs.turnControllerKd);
    }
    @Override
    public void execute(){
        PIDs.driveControllerKp = SmartDashboard.getNumber("P Gain", 0);
        PIDs.driveControllerKi = SmartDashboard.getNumber("I Gain", 0);
        PIDs.driveControllerKd = SmartDashboard.getNumber("D Gain", 0);
        PIDs.driveControllerKp = SmartDashboard.getNumber("P Gain", 0);
        PIDs.driveControllerKi = SmartDashboard.getNumber("I Gain", 0);
        PIDs.driveControllerKd = SmartDashboard.getNumber("D Gain", 0);
    }
}
