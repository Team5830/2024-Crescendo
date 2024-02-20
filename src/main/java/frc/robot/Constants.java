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
    public static final double driveControllerKp = 0.002;
    public static final double driveControllerKi = 0;
    public static final double driveControllerKd = 0;
    public static final double turnControllerKp = .001;
    public static final double turnControllerKi = 0;
    public static final double turnControllerKd = 0.0005;

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
    public static final boolean invertNavX = true;
  }

  public static final class Joystick {
    public static final int port = 0;

    public static final double xRateLimit = 3;
    public static final double yRateLimit = 3;
    public static final double rotRateLimit = 3;
  }

  public static final class Intake {
    public static final double firstIntakSspeed = 0.4;
    public static final double P = 0.1;
    public static final double I = 0.0;
    public static final double D = 0.0;
    public static final double F = 0.0;
    public static final double zI = 0;
    public static final double kMaxOutput = 0.4;
    public static final double kMinOutput = 0.1;
    public static final int motorChanel = 21;
  }

  public static final class Flywheel {
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
    public static final int motorChanel = 6;
  }

  public static final class Arm {
    public static final double kP = 0.0012;
    public static final double kI = 0.000000;
    public static final double kD = 0.04;
    public static final double kIz = 0;
    public static final double kFF = 0;
    public static final double kMaxOutput = 0.7;
    public static final double kMinOutput = -0.7;
    public static final double speedTolerance = 50.0;
    public static final double maxOutput = .4;
    public static final double minOutput = -.4;
    public static final float forwardLimit = 300;
    public static final float reverseLimit = 0;
    public static final double tolerance = 5.0;
    public static final int motorChanel = 60;

    public static final class Position1 { // POSITION 1 NAME HERE
      public static final double armAngle = 0.0;
    }

    public static final class Position2 { // POSITION 2 NAME HERE
      public static final double armAngle = 49.1;
    }

    public static final class Position3 { // POSITION 3 NAME HERE
      public static final double armAngle = 236.0;
    }

    public static final class Position4 { // POSITION 4 NAME HERE
      public static final double armAngle = 278.0;
    }
  }
}
