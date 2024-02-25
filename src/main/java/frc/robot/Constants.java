package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveTrain {
    public static final double maxSpeed = 3.0; // 3 meters per second
    public static final double maxAngularVelocity = 100; // revolutions per second
    public static final double maxAngularAcceleration = 10; // revolutions per second squared
    public static final double AngleTolerance = 5;
    public static final double turnarget = 90;

    // PIDs
    public static final double driveControllerKp = 0.12;
    public static final double driveControllerKi = 0;
    public static final double driveControllerKd = 0.0;
    public static final double turnControllerKp = .02;
    public static final double turnControllerKi = 0;
    public static final double turnControllerKd = 0.002;

    // Feedforward gains
    public static final double driveFeedforwardStatic = 0.01;
    public static final double driveFeedforwardVelocity = 0;
    public static final double turnFeedforwardStatic = 0;
    public static final double turnFeedforwardVelocity = 0.0;

    // Channels
    public static final int frontLeftDriveChannel = 1;
    public static final int frontLeftTurnChannel = 2;
    public static final int frontRightDriveChannel = 4;
    public static final int frontRightTurnChannel = 3;
    public static final int backLeftDriveChannel = 7;
    public static final int backLeftTurnChannel = 8;
    public static final int backRightDriveChannel = 5;
    public static final int backRightTurnChannel = 6;
    // 
    public static final boolean invertNavX = false;
  }

  public static final class controller {
    public static final int xboxPort = 0;
    public static final int flyskyPort = 1;

    public static final double xRateLimit = 3;
    public static final double yRateLimit = 3;
    public static final double rotRateLimit = 3;
  }

  public static final class intake {
    public static final double firstIntakTopSspeed = 0.4;
    public static final double firstIntakBottomSspeed = 0.15;
    public static final double P = 0.1;
    public static final double I = 0.0;
    public static final double D = 0.0;
    public static final double F = 0.0;
    public static final double zI = 0;
    public static final double kMaxOutput = 0.4;
    public static final double kMinOutput = 0.1;
    public static final int motorChannel = 11;
    public static final int motorChanneltop = 9;
  }

  public static final class flywheel {
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
    public static final int motorChanelTop = 14;
    public static final int motorChanelBottom = 17;
  }

  public static final class arm {
    public static final double kP = 0.1;
    public static final double kI = 0.000000;
    public static final double kD = 7;
    public static final double kIz = 0;
    public static final double kFF = 0;
    public static final double kMaxOutput = 0.7;
    public static final double kMinOutput = -0.7;
    public static final double speedTolerance = 50.0;
    public static final double maxOutput = .4;
    public static final double minOutput = -.4;
    public static final float forwardLimit = 2.2f;
    public static final float reverseLimit = -6.8f;
    public static final double tolerance = 5.0;
    public static final int motorChanel = 12;
    public static final double incrementValue = .2;

    public static final class Position1 { // Down Position
      public static final double armAngle = -6.9;
    }

    public static final class Position2 { // Middle Position
      public static final double armAngle = -4;
    }

    public static final class Position3 { // Up position
      public static final double armAngle = 0;
    }
  }

  public static final class TurnPID {
    public static final double P = 0.08;
    public static final double I = 0.0;
    public static final double D = 0.0;
    public static final double f = 0.0;
    public static final double Tolerance = 2.0; // Measured in degrees
    public static final double TurnRateTolerance = 10; //Degrees per second
  }
  public static final class climber{
    public static final float upLeftHeight = 96.00000000000000000000000f;
    public static final float downLeftHeight = -5f; 
       public static final float upRightHeight = 75.00000000000000000000000f;
    public static final float downRightHeight = -5f; 
    public static final int leftMotorChanel = 16;
    public static final int rightMotorChanel = 15;
    public static final double kP = 0.02;
    public static final double kI = 0.000000;
    public static final double kD = 0.0;
    public static final double kIz = 0;
    public static final double kFF = 0;
    public static final double maxOutput = .4;
    public static final double minOutput = -.4;
    public static final double tolerance = 5.0;
  }
}
