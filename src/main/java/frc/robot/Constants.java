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
    public static final double maxAngularVelocity = Math.PI; // 1/2 rotation per second
    public static final double maxAngularAcceleration = 2 * Math.PI; // radians per second squared

    // PIDs
    public static final double driveControllerKp = 1;
    public static final double driveControllerKi = 0;
    public static final double driveControllerKd = 0;
    public static final double turnControllerKp = 1;
    public static final double turnControllerKi = 0;
    public static final double turnControllerKd = 0;

    // Feedforward gains
    public static final double driveFeedforwardStatic = 1;
    public static final double driveFeedforwardVelocity = 3;
    public static final double turnFeedforwardStatic = 1;
    public static final double turnFeedforwardVelocity = 0.5;

    // Channels
    public static final int gyroChannel = 0;
    public static final int frontLeftDriveChannel = 1;
    public static final int frontLeftTurnChannel = 2;
    public static final int frontRightDriveChannel = 3;
    public static final int frontRightTurnChannel = 4;
    public static final int backLeftDriveChannel = 5;
    public static final int backLeftTurnChannel = 6;
    public static final int backRightDriveChannel = 7;
    public static final int backRightTurnChannel = 8;
  }

  public static final class Joystick {
    public static final int port = 0;

    public static final double xRateLimit = 3;
    public static final double yRateLimit = 3;
    public static final double rotRateLimit = 3;

  }
}
