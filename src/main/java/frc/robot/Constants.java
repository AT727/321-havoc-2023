/* (C) Robolancers 2024 */
package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class Arm {
    public static final class Anchor {
      public static final int kAnchorPort = 22;

      public static final boolean kInverted = true;
      public static final double kZeroPosition = 13;
      public static final double kMinAngle = 13;
      public static final double kMaxAngle = 90;
      public static final boolean kEnableSoftLimit = true;
      public static final double kMaxOutput = 0.5; // going up
      public static final double kMinOutput = -0.4; // going down
      public static final int kCurrentLimit = 60; // 40 to 60
      public static final double kAnchorLength = 40.3; // in

      public static final class PID {
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final int kSlot = 0;
      }

      public static final class FF {
        public static final double ks = 0;
        public static double kg = 0.0; // gravity FF most likely only tune this gain
        public static final double kv = 0;
        public static final double ka = 0;
        public static ArmFeedforward ANCHOR_FEEDFORWARD = new ArmFeedforward(ks, kg, kv, ka);
      }

      public static final class MP {
        public static final double maxVel = 1.0;
        public static final double maxAccel = 3.0;
        public static final TrapezoidProfile.Constraints ANCHOR_CONSTRAINTS =
            new TrapezoidProfile.Constraints(maxVel, maxAccel);
      }

      public static final class Conversions {
        public static final double kRatio = (90.0 - 13.0) / (27.0);
      }
    }

    public static final class Floating {
      public static final int kFloatingPort = 23;

      public static final boolean kInverted = true;
      public static final double kZeroPosition = 0;
      public static final double kMinAngle = 22.0;
      public static final double kMaxAngle = 180.0;
      public static final boolean kEnableSoftLimit = true;
      public static final double kMaxOutput = 0.5; // going up
      public static final double kMinOutput = -0.5; // going down
      public static final int kCurrentLimit = 60; // 40 to 60
      public static final double kFloatingLength = 28.0; // in

      public static final class PID {
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final int kSlot = 0;
      }

      public static final class FF {
        public static final double ks = 0;
        public static final double kg = 0; // gravity FF most likely only tune this gain
        public static final double kv = 0;
        public static final double ka = 0;
        public static final ArmFeedforward FLOATING_FEEDFORWARD =
            new ArmFeedforward(ks, kg, kv, ka);
      }

      public static final class MP {
        public static final double maxVel = 1.0;
        public static final double maxAccel = 1.0;
        public static final TrapezoidProfile.Constraints FLOATING_CONSTRAINTS =
            new TrapezoidProfile.Constraints(maxVel, maxAccel);
      }

      public static final class Conversions {
        public static final double kRatio = 360;
      }

      // TODO fix this from arm setpoint vel & pos to goal setpoint for inverse kinematics
    }
  }
}
