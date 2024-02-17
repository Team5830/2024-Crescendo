package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.Constants.DriveTrain;
public class testTurning extends Command {
    private SwerveDrive m_swervedrive;
    private double p,i,d,target;
    public testTurning(SwerveDrive m_swervedrive){
        m_swervedrive = m_swervedrive;
        addRequirements(m_swervedrive);
    }

    @Override 
    public void initialize(){
        p = SmartDashboard.getNumber("TurnP", DriveTrain.driveControllerKp);
        i = SmartDashboard.getNumber("TurnI", DriveTrain.driveControllerKi);
        d = SmartDashboard.getNumber("TurnD", DriveTrain.driveControllerKd);
        target = SmartDashboard.getNumber("TurnTarget", 0);
        m_swervedrive.m_frontLeft.updatePIDValues(p,i,d);
        m_swervedrive.m_frontRight.updatePIDValues(p,i,d);
        m_swervedrive.m_backLeft.updatePIDValues(p,i,d);
        m_swervedrive.m_backRight.updatePIDValues(p,i,d);
        m_swervedrive.m_frontLeft.setTurnTarget(target);
    }

}
