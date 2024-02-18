package frc.robot.commands;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.Constants.DriveTrain;
public class testTurning extends Command {
    private final SwerveDrive m_swerveDrive;
    private double p,i,d,target;
    public testTurning(SwerveDrive swerveDrive){
        m_swerveDrive = swerveDrive;
        addRequirements(m_swerveDrive);
    }

    @Override 
    public void initialize(){
        
        p = SmartDashboard.getNumber("TurnP", DriveTrain.driveControllerKp);
        i = SmartDashboard.getNumber("TurnI", DriveTrain.driveControllerKi);
        d = SmartDashboard.getNumber("TurnD", DriveTrain.driveControllerKd);
        target = SmartDashboard.getNumber("TurnTarget", 0);
        m_swerveDrive.m_frontLeft.updatePIDValues(p,i,d);
        m_swerveDrive.m_frontRight.updatePIDValues(p,i,d);
        m_swerveDrive.m_backLeft.updatePIDValues(p,i,d);
        m_swerveDrive.m_backRight.updatePIDValues(p,i,d);
        m_swerveDrive.m_frontLeft.setTurnTarget(target);
    }

}
