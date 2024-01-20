// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class CANBusID {
        public static final int kLeftMotor1 = 10;
        public static final int kRightMotor1 = 12;
        public static final int kLeftMotor2 = 13;
        public static final int kRightMotor2 = 11;
        //public static final int kLeftFlywheel = 6;// left flywheel motor
        public static final int kLeftFlywheel = 1;// right flywheel motor
        public static final int dintakemotor = 21;// intake
        public static final int conveyor1 = 20;// conveyor 1
        public static final int conveyor2 = 2;// shooter conveyor
        public static final int dexotor = 9;// extend intake
        public static final int climberMotorup = 3;// climber motor 1
    }

    public static final class Ports {
        public static final int LeftDriveEncoder1 = 0;
        public static final int LeftDriveEncoder2 = 1;
        public static final int RightDriveEncoder1 = 2;
        public static final int RightDriveEncoder2 = 3;
        public static final int Conveyor1ballsensor = 8;
        public static final int Conveyor2ballsensor = 9;
    }

    public static final class ConveyorC {
        public static final double speed = 0.5;
        public static final double DownforShot = 4.0;
        public static final double UpforShot = 10.5;
        public static final double currentLimit = 30.0;
        public static final double kP = 0.3;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kIz = 0;
        public static final double kFF = 0;
        public static final double kMaxOutput = 0.7;
        public static final double kMinOutput = -0.7;
    }

    public static final class MovePID {
        public static final double P = 0.1;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double f = 0.0;
        public static final double MaxAlignSpeed = 24; // Inches per second
        public static final double AlignTolerance = 12; // Inches
    }

    public static final class TurnPID {
        public static final double kTurnP = 2e-2;
        public static final double kTurnI = 1e-3;
        public static final double kTurnD = 0.0001;
        public static final double kTurnF = 0.4;
        public static final double kTurnToleranceDeg = 2.0;
        public static final double kTurnRateToleranceDegPerS = 100.0;

    }

    public static final class DriveC {
        public static final double reducedMaxSpeed = 0.7;
        public static final double MaxSpeed = 1.0;
        public static final boolean SquareInputs = true;
        public static final int kEncoderCPR = 2048;
        public static final double kWheelCircInches = 19.0;
        public static final double distancePerPulse = (kWheelCircInches) / (double) kEncoderCPR;// Gear
    }

    public static final class FlywheelC {
        public static final int waitforshootersecs = 10;
        public static final double feedmotorspeed = 0.5;
        public static final double shootermotorspeed = 1600;
        public static final double kP = 0.0012;
        public static final double kI = 0.000000;
        public static final double kD = 0.04;
        public static final double kIz = 0;
        public static final double kFF = 0;
        public static final double kMaxOutput = 0.7;
        public static final double kMinOutput = -0.7;
        public static final double speedTolerance = 50.0;
    }

    public static final class ClimberC {
        public static final double climberSpeed = 1.0;
        public static final double climberforwardlimit = 475;
        public static final double climberreverselimit = 0;
    }

    public static final class firstIntake {
        public static final double firstIntakespeed = 0.4;
        public static final double ExtendSpeed = 0.3;
        public static final double ExtendDistance = 118;// how far to turn motor to extend arm
        public static final double ExtendminDistance = 5;// how far to turn motor to extend arm
        public static final double P = 0.1;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double F = 0.0;
        public static final double zI = 0;
        public static final double kMaxOutput = 0.4;
        public static final double kMinOutput = 0.1;
    }

    public static final class buttonsLeftjoy {
        public static final int halfspeedButton = 5;
        public static final int toggleIntakeExtend = 6;
        public static final int toggleIntake = 2;
        public static final int toggleconveyor1 = 7;
        public static final int toggleconveyor2 = 1;

    }

    public static final class buttonsRightjoy {
        public static final int halfspeedButton = 5;
        public static final int pickupButton = 3;
        public static final int shootButton = 1;
        public static final int pickupOffButton = 4;
        public static final int moveButton = 1;
        public static final int turnleftButton = 3;
        public static final int turnrightButton = 4;
    }

    public static final class Conveyspeed {
        public static final double conveyorspeed1 = 0.3;
        public static final double conveyorspeed2 = 0.5;
    }
}