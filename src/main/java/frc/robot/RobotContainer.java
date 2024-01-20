// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.buttonsRightjoy;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //public final DriveTrain m_drivetrain = new DriveTrain();
  //private final FirstIntake m_intake = new FirstIntake();
  public final Flywheel m_flywheel = new Flywheel();
  //private final Joystick m_leftJoy = new Joystick(0);
  private final Joystick m_rightJoy = new Joystick(1);
  //private final Climber m_climber = new Climber();
  //private final Conveyor m_conveyor = new Conveyor();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();
   // SendableRegistry.setName(m_drivetrain, "DriveTrain", "DriveTrain");
    //SendableRegistry.setName(new Turn(90, m_drivetrain), "Turn Right command");
    //SendableRegistry.setName(new Turn(-90, m_drivetrain), "Turn Left command");

    // SendableRegistry.setName(new InstantCommand(m_climber::climberMoter1on),
    // "Turn Climber1 on");
    // SendableRegistry.setName(new InstantCommand(m_climber::reverse_Motor1),
    // "Reverse Climber1");
    //SendableRegistry.setName(new Conv1(m_conveyor), "Conveyor1On");
    //SendableRegistry.setName(new Conv2(m_conveyor), "Conveyor2On");
    //SendableRegistry.setName(new InstantCommand(m_conveyor::conveyor1Reversed), "Reverse Conveyor1");
    //SendableRegistry.setName(new InstantCommand(m_conveyor::conveyor2Reversed), "Reverse Conveyor2");
    SendableRegistry.setName(m_flywheel, "Flywheel");
    //SendableRegistry.setName(m_conveyor, "Conveyor");
    //SendableRegistry.setName(m_intake, "Intake");
    // SendableRegistry.setName(m_climber, "Climber");

    //m_drivetrain.setDefaultCommand(new Drive(m_drivetrain, () -> m_leftJoy.getY(), () -> m_rightJoy.getY()));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
   private void configureButtonBindings() {
    // Lower max speed
    // new JoystickButton(m_leftJoy, buttonsLeftjoy.halfspeedButton).whenPressed(()
    // -> m_drivetrain.toggleMaxSpeed());
    // new JoystickButton(m_leftJoy, buttonsRightJoy.intakeoffButton).whenPressed(()
    // -> m_intake.);
     //new JoystickButton(m_rightJoy, buttonsRightjoy.pickupButton);
        //.whenPressed(new Pickup(m_intake, m_conveyor, m_flywheel));
    new JoystickButton(m_rightJoy, buttonsRightjoy.shootButton).whileTrue(new Shoot(m_flywheel));
    //new JoystickButton(m_rightJoy, buttonsRightjoy.pickupOffButton).whenPressed(new PickupOff(m_intake, m_conveyor));

    /*
     * new JoystickButton(m_leftJoy, buttonsLeftjoy.toggleIntake).whenPressed(()->
     * m_intake.toggleFirstIntake());
     * new JoystickButton(m_rightJoy, buttonsRightjoy.moveButton).whenPressed( new
     * Move(100.0,m_drivetrain).withTimeout(5));
     * new JoystickButton(m_rightJoy,
     * buttonsRightjoy.turnrightButton).whenPressed(new Turn(90,
     * m_drivetrain).withTimeout(5));
     * new JoystickButton(m_rightJoy,
     * buttonsRightjoy.turnleftButton).whenPressed(new Turn(-90,
     * m_drivetrain).withTimeout(5));
     * new JoystickButton(m_leftJoy,
     * buttonsLeftjoy.toggleIntakeExtend).whenPressed(()->
     * m_intake.toggleExtension());
     * new JoystickButton(m_leftJoy,
     * buttonsLeftjoy.toggleconveyor1).whenPressed(()->
     * m_conveyor.toggleconveyor1());
     * new JoystickButton(m_leftJoy,
     * buttonsLeftjoy.toggleconveyor2).whenPressed(()->
     * m_conveyor.toggleconveyor2());
     */

    // SmartDashboard.putData("Climber Up", new
    // InstantCommand(m_climber::climberMoter1on));
     //SmartDashboard.putData("Climber Off", new
    // InstantCommand(m_climber::climberMoter1off));
    // SmartDashboard.putData("Climber Down", new
    // InstantCommand(m_climber::reverse_Motor1));
    // ShuffleboardTab FlywheelControl = Shuffleboard.getTab("Flywheel");
    //ShuffleboardTab DriveTrainControl = Shuffleboard.getTab("Drivetrain");
    //DriveTrainControl.add("Drivetrain", m_drivetrain);
    //ShuffleboardTab ClimberControl = Shuffleboard.getTab("Climber");
    //ShuffleboardTab IntakeControl = Shuffleboard.getTab("Intake");
    SmartDashboard.putData("Flywheel On", new Flywheel_test(m_flywheel));
    SmartDashboard.putData("Easy Shooter", new InstantCommand(m_flywheel::shooterGo));
    SmartDashboard.putData("Flywheel Off", new InstantCommand(m_flywheel::shooteroff));
    /*SmartDashboard.putData("IntakeRetractOFf", new InstantCommand(m_intake::stopRetract));
    SmartDashboard.putData("Intake Down", new InstantCommand(m_intake::intakeDown));
    SmartDashboard.putData("Intake Up", new InstantCommand(m_intake::intakeUp));
    SmartDashboard.putData("Toggle extend", new InstantCommand(m_intake::toggleExtension));
    SmartDashboard.putData("Intake On", new InstantCommand(m_intake::startFirstIntake));
    SmartDashboard.putData("Intake Off", new InstantCommand(m_intake::stopFirstIntake));
    SmartDashboard.putData("Intake Toggle", new InstantCommand(m_intake::toggleFirstIntake));
    SmartDashboard.putData("Reset Gyro", new InstantCommand(m_drivetrain::resetHeading));
    SmartDashboard.putData("Reset DT Encoders", new InstantCommand(m_drivetrain::resetEncoders));
    SmartDashboard.putData("Intake Reverse", new InstantCommand(m_intake::reverseFirstIntake));
    SmartDashboard.putData("Climber Up", new InstantCommand(m_climber::climber_up));
    SmartDashboard.putData("Climber Down", new InstantCommand(m_climber::climber_down));
    SmartDashboard.putData("Climber Off", new InstantCommand(m_climber::climberMoter1off));
    SmartDashboard.putData("Conveyor2 Reverse", new InstantCommand(m_conveyor::conveyor2Reversed));
    SmartDashboard.putData("Conveyor1 Toggle", new InstantCommand(m_conveyor::toggleconveyor1));
    SmartDashboard.putData("Conveyor2 Toggle", new InstantCommand(m_conveyor::toggleconveyor2));
    SmartDashboard.putData("Pickup", new Pickup(m_intake, m_conveyor, m_flywheel));
    SmartDashboard.putData("Pickup Off", new PickupOff(m_intake, m_conveyor));
    SmartDashboard.putData("Conv2down", new InstantCommand(m_conveyor::conv2down));
    SmartDashboard.putData("Conv2up", new InstantCommand(m_conveyor::conv2up));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
 /*  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
        new InstantCommand(m_flywheel::shooterGo),
        new Pause(2.0),
        new InstantCommand(m_conveyor::conveyor2ON),
        // new Shoot(m_flywheel, m_conveyor),
        new InstantCommand(m_drivetrain::toggleMaxSpeed),
        new Pause(2.0),
        new Move(-60, m_drivetrain),
        new InstantCommand(m_drivetrain::toggleMaxSpeed),
        new InstantCommand(m_flywheel::shooteroff),
        new InstantCommand(m_conveyor::conveyor2OFF),
        new InstantCommand(m_intake::intakeDown));
  }*/

  }
}