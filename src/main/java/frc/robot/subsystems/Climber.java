package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import com.revrobotics.CANSparkBase.ControlType;
public class Climber extends SubsystemBase {

  private CANSparkMax m_leftmotor;
  public RelativeEncoder m_leftencoder;
  private SparkPIDController m_leftpidController;
  private double lefttargecko;  
  private CANSparkMax m_rightmotor;
  public RelativeEncoder m_rightencoder;
  private SparkPIDController m_rightpidController;
  private double righttargecko;

  public Climber() {
    try {
      m_leftmotor = new CANSparkMax(Constants.climber.leftMotorChanel, CANSparkMax.MotorType.kBrushless);
      m_leftmotor.restoreFactoryDefaults();
      m_leftencoder = m_leftmotor.getEncoder();
      m_leftencoder.setPositionConversionFactor(1);
      m_leftencoder.setPosition(0.0);
      m_leftmotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
      m_leftmotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
      m_leftmotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.climber.upLeftHeight);
      m_leftmotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, Constants.climber.downLeftHeight-5);

      m_rightmotor = new CANSparkMax(Constants.climber.rightMotorChanel, CANSparkMax.MotorType.kBrushless);
      m_rightmotor.restoreFactoryDefaults();
      m_rightencoder = m_rightmotor.getEncoder();
      m_rightencoder.setPositionConversionFactor(1);
      m_rightencoder.setPosition(0.0);
      m_rightmotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
      m_rightmotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
      m_rightmotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.climber.upRightHeight);
      m_rightmotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, Constants.climber.downRightHeight-5);

      m_leftpidController = m_leftmotor.getPIDController();
      m_leftpidController.setP(Constants.climber.kP);
      m_leftpidController.setI(Constants.climber.kI);
      m_leftpidController.setD(Constants.climber.kD);
      m_leftpidController.setFF(Constants.climber.kFF);
      m_leftpidController.setOutputRange(Constants.climber.minOutput, Constants.climber.maxOutput);
      leftmove(0);
      m_rightpidController = m_rightmotor.getPIDController();
      m_rightpidController.setP(Constants.climber.kP);
      m_rightpidController.setI(Constants.climber.kI);
      m_rightpidController.setD(Constants.climber.kD);
      m_rightpidController.setFF(Constants.climber.kFF);
      m_rightpidController.setOutputRange(Constants.climber.minOutput, Constants.climber.maxOutput);
      rightmove(0);
      // m_karmoterPID.setPositionPIDWrappingMaxInput(180);
      // m_karmoterPID.setPositionPIDWrappingMinInput(-180);
      // m_karmoterPID.setPositionPIDWrappingEnabled(true);

    } catch (RuntimeException ex) {
      DriverStation.reportError("Error configuring Climber: " + ex.getMessage(), true);
    }

  }

  public void leftmove(double height) {
    lefttargecko = height;
    m_leftpidController.setReference(lefttargecko, ControlType.kPosition);
    DriverStation.reportWarning(String.format("Climberr Position %f", m_leftencoder.getPosition()), false);
  }
   
  public void rightmove(double height) {
    righttargecko = height;
    m_rightpidController.setReference(righttargecko, ControlType.kPosition);
    DriverStation.reportWarning(String.format("Climberrm Position %f", m_rightencoder.getPosition()), false);
  }


  public boolean lefatTarget() {
    double curposition = m_leftencoder.getPosition();
    DriverStation.reportWarning(String.format("Position: %f", curposition), false);
    if (Math.abs(curposition - lefttargecko) <= Constants.climber.tolerance) {
      DriverStation.reportWarning("True", false);
      return true;
    } else {
      DriverStation.reportWarning(String.format("false: %f", Math.abs(curposition - lefttargecko)), false);
      return false;
    }
  }

  public boolean rightOnTarget() {
    double curposition = m_rightencoder.getPosition();
    DriverStation.reportWarning(String.format("Position: %f", curposition), false);
    if (Math.abs(curposition - righttargecko) <= Constants.climber.tolerance) {
      DriverStation.reportWarning("True", false);
      return true;
    } else {
      DriverStation.reportWarning(String.format("false: %f", Math.abs(curposition - righttargecko)), false);
      return false;
    }
  }


  public double leftPosition() {
    SmartDashboard.putNumber("Climberzzz", m_leftencoder.getPosition());
    // SmartDashboard.putNumber("AFL", Constants.climber.upHeight);
    // SmartDashboard.putNumber("ALR", Constants.climber.downHeight);
    return m_leftencoder.getPosition();
  }

  public double righgtPosition() {
    SmartDashboard.putNumber("Climberzzz", m_rightencoder.getPosition());
    // SmartDashboard.putNumber("AFL", Constants.climber.upHeight);
    // SmartDashboard.putNumber("ALR", Constants.climber.downHeight);
    return m_rightencoder.getPosition();
  }

  public boolean Safe() {
    //Always stay safe !
      return true;
    }

  public void rightStop() {
    // armMotorController.set(0);
    m_rightmotor.stopMotor();
    SmartDashboard.putNumber("ClimberPosition", m_rightencoder.getPosition());
  }

  public void leftStop() {
    // armMotorController.set(0);
    m_leftmotor.stopMotor();
    SmartDashboard.putNumber("ClimberPosition", m_leftencoder.getPosition());
  }

  public void lencerement() {
    if (lefttargecko + .5 <= Constants.climber.upLeftHeight) {
      lefttargecko = lefttargecko + .5;
      m_leftpidController.setReference(lefttargecko, ControlType.kPosition);
    }
  }

  public void rincerement() {
    if (righttargecko + .5 <= Constants.climber.upRightHeight) {
      righttargecko = righttargecko + .5;
      m_rightpidController.setReference(righttargecko, ControlType.kPosition);
    }
  }

  public void lecroment() {
    if (lefttargecko - .5 >= Constants.climber.downLeftHeight) {
      lefttargecko = lefttargecko - .5;
      m_leftpidController.setReference(lefttargecko, ControlType.kPosition);
    }
  }

  public void ricroment() {
    if (righttargecko - .5 >= Constants.climber.downRightHeight) {
      righttargecko = righttargecko - .5;
      m_rightpidController.setReference(righttargecko, ControlType.kPosition);
    }
  }

  public  void useUpPosition() {
    leftmove(Constants.climber.upLeftHeight);
    rightmove(Constants.climber.upRightHeight);
  }


  public  void useDownPosition() {
    leftmove(0);
    rightmove(0);
  }

    public  void manuallyLowerLeftLimit() {
    leftmove(Constants.climber.downLeftHeight);
  }
      public  void manuallyLowerRightLimit() {
    rightmove(Constants.climber.downRightHeight);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Climber Position", m_leftencoder.getPosition());
    SmartDashboard.putNumber("Right Climber Position", m_rightencoder.getPosition());
    SmartDashboard.getNumber("Left Target", lefttargecko);
    SmartDashboard.getNumber("Right Target", righttargecko);
  }
}