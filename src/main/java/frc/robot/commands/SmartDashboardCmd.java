package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class SmartDashboardCmd extends Command {
    public SmartDashboardCmd(SwerveDrive m_sdrive){
        
    }
    @Override
    public void execute(){
        PIDs.driveControllerKp = SmartDashboard.getNumber("DriveP", 0);
        PIDs.driveControllerKi = SmartDashboard.getNumber("DriveI", 0);
        PIDs.driveControllerKd = SmartDashboard.getNumber("DriveD", 0);
        PIDs.driveControllerKp = SmartDashboard.getNumber("TurnP", 0);
        PIDs.driveControllerKi = SmartDashboard.getNumber("TurnI", 0);
        PIDs.driveControllerKd = SmartDashboard.getNumber("TurnD", 0);
    }
}
