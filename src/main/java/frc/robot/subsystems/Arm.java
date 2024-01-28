package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase  {

    private CANSparkMax armMotorController;
    public RelativeEncoder armEncoder;
    private SparkPIDController armPID;
    private double karget;
    private double P,I,D;
    private double kFF;
    public Arm() {
        try{
            armMotorController = new CANSparkMax(Constants.Arm.channel , CANSparkMax.MotorType.kBrushless);
            armMotorController.restoreFactoryDefaults();
            armEncoder = armMotorController.getEncoder();
            //armEncoder.setPositionConversionFactor(8);
            armEncoder.setPosition(0.0);
            //armMotorController.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
            //armMotorController.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);    
            //armMotorController.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.Arm.ForwardLimit);
            //armMotorController.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, Constants.Arm.ReverseLimit);
            
            armPID = armMotorController.getPIDController();
            armPID.setP(Constants.Arm.controllerKp);
            P = Constants.Arm.controllerKp;
            armPID.setI(Constants.Arm.controllerKi);
            I = Constants.Arm.controllerKi;
            armPID.setD(Constants.Arm.controllerKd);
            D = Constants.Arm.controllerKd;
            armPID.setFF(Constants.Arm.controllerKf);
            kFF = Constants.Arm.controllerKf;
            armPID.setOutputRange(Constants.Arm.MinOutput, Constants.Arm.MaxOutput);
            move(0);
            
            }catch (RuntimeException ex) {
                DriverStation.reportError("Error Configuring Drivetrain" + ex.getMessage(), true);
            }
        
    }

    public void updatePID(){
        double p = SmartDashboard.getNumber("Arm P", Constants.Arm.controllerKp);
        double i = SmartDashboard.getNumber("Arm I", Constants.Arm.controllerKi);
        double d = SmartDashboard.getNumber("Arm D", Constants.Arm.controllerKd);
        double ff = SmartDashboard.getNumber("Arm FF", Constants.Arm.controllerKf);
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != P)) { armPID.setP(p); P = p;  }
        if((i != I)) { armPID.setI(i); I = i; }
        if((d != D)) { armPID.setD(d); D = d; }
        if((ff != kFF)) { armPID.setFF(ff); kFF = ff; }  
        DriverStation.reportWarning("Updated PID", false);
    }
    
    public void move(double degrees) {
        karget = degrees;
        armPID.setReference(karget, ControlType.kPosition);
        DriverStation.reportWarning(String.format("Arm Position %f",armEncoder.getPosition()),false);
    }
    public boolean AtTarget(){
        double curposition = armEncoder.getPosition();
        DriverStation.reportWarning(String.format("Position: %f",curposition),false);
        if ( Math.abs(curposition - karget) <=Constants.Arm.Tolerance){
            DriverStation.reportWarning("True",false);
            return true;
        }else{
            DriverStation.reportWarning(String.format("false: %f",Math.abs(curposition - karget)),false);
            return false;
        }
    }

    public double Position(){
        SmartDashboard.putNumber("Arm", armEncoder.getPosition());
        return armEncoder.getPosition();
    }

    public void Stop(){
        //armMotorController.set(0);
        armMotorController.stopMotor();
        SmartDashboard.putNumber("ArmPosition", armEncoder.getPosition());
    }
    public void increment(){
        if (karget + 3 <= Constants.Arm.ForwardLimit){
            karget = karget + 3;
            armPID.setReference(karget, ControlType.kPosition);
        }
    }
    public void decrement(){
        if (karget - 3 >= Constants.Arm.ReverseLimit){
            karget = karget - 3;
            armPID.setReference(karget, ControlType.kPosition);
        }
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("ArmPosition", armEncoder.getPosition());
        //SmartDashboard.getNumber("karget", karget );
    }

 
}

