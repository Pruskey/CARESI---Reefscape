package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.wpilibj.Encoder;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

/* public static RobotConfig config;

  static {
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
  }
  */

  public static class OperatorConstants {
    public static final int kDriverControllerID = 0;
    public static final int kShooterControllerID = 1;
  }

  public static class DrivetrainConstants {
    public static final int kLeftLeaderID = 1;
    public static final int kLeftFollowerID = 2;
    public static final int kRightLeaderID = 3;
    public static final int kRightFollowerID = 4;
    public static final int kPigeonID = 9;
  }

  public static class ArmConstants {
    public static final int armMotorID = 5;
    public static final int climbMotorID = 8;
    public static final int encoderIDa = 2;
    public static final int encoderIDb = 3;
  }

  public static class IntakeConstants {
    public static final int rotateMotorID = 6;
    public static final int intakeMotorID = 7;
  }

  public static class PathPlannerConstants {
    public static final double kMaxVelocity = 4.0; // Adjust according to your robot's max velocity in meters per second
    public static final double kMaxAcceleration = 3.0; // Adjust according to your robot's max acceleration in meters per second squared

    // Add more PathPlanner specific constants as needed
  }
}
