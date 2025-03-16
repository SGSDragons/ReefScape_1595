// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class OperatorConstants {

      public static final int driverControllerPort = 0;
      public static final int operatorControllerPort = 1;

      public static double stickDeadband() {
          return Preferences.getDouble("Deadband", 0.1);
      }
  }
  //Hardware IDs for parts on robot (excluding drivetrain).
  public static final class HardwareID {

      // All the CANBus and Channel IDs used on the robot. If any of these IDs are repeated,
      // then things won't work correctly
      //
      // Can ID -1 denotes numbers that have yet to be programmed on devices.
      // 
      // IDs 3-14 are used by the swerve modules. Do NOT reuse here.

      public static final class Lift {
        public static final int RightMotorCanId = 16;
        public static final int LeftMotorCanId = 2;
        public static final int FlipperCanId = 20;
        public static final int WireSpoolCanId = -1;

        public static final int LimitSwitchChannelId = 0;
      }

      public static final class Carriage {
        public static final int WheelsMotorCanId = 21; // Wheels that transport a coral
        public static final int directionServoChannelId = 0; // The PWM port that the servo plugs into 
      }

      public static final class Climb {
        public static final int MotorCanId = 15; // Climber motor
      }

      public static final class CoralIntake {
          public static final int ExtenderCanId = -1; // The motor that extends and retracts the coral arms
          public static final int LeftWheelsCanId = -1; // The motor that spins the left-side intake wheels
          public static final int RightWheelsCanId = -1; // The motor that spins the right-side intake wheels
          public static final int TopWheelCanId = -1; // The motor that spins the top intake bar  
      }

      public static final class Algae {
        public static final int FourBarCanId = 18;
        public static final int RollerCanId = 1;
      }
  }

  public static final class SystemToggles {
    public static final boolean useCompleteAuto = false;
  }

    public class LiftConstants{

      public static final double kS = 0.8;
      public static final double kG = 0;
      public static final double kP = 0.5;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kGravity = 0;

      public static final double intakeheight = 27.63;
      public static final double Ground = 0;
      public static final double Shelf = 16.6;
      public static final double Low = 16.03;
      public static final double Medium = 38.86;
      public static final double High = 60;

      public static final double TopLimit = 77;

      public static final double TopAngle = 3;
      public static final double DefaultAngle = 0;
      public static final double IntakeAngle = 1;

      public static final double WiretoLiftRatio = 1;

    }

    public class CarriageConstants {

      public static final double outtakeSpeed = 0.5;

      public static final double pointRight = -0.1;
      public static final double middle = 0;
      public static final double pointLeft = 0.1;

    }

    public class CoralIntakeConstants {

      public static final double kS = 0.5;
      public static final double kG = 0;
      public static final double kP = 0;
      public static final double kI = 0;
      public static final double kD = 0;

      public static final double Extend = 1;
      public static final double Retract = 0;
      public static final double Intake = 0.5;
      public static final double Outtake = -0.5;

    }

    public class AlgaeContants {

      public static final double kS = 0.5;
      public static final double kG = 0;
      public static final double kP = 0.8;
      public static final double kI = 0;
      public static final double kD = 0.1;

      public static final double Extend = -6.14;
      public static final double Retract = 0;

      //below 2.75 to stop motor
      public static final double CurrentLimit = 3;

    }

    public class Reefscape {
      public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
      }

      public static final Translation2d redReef = new Translation2d(13, 4);
      public static final Translation2d blueReef = new Translation2d(4.5, 4);
      public static Translation2d getReefLocation() {
        return isRedAlliance() ? redReef : blueReef; 
      }

      public static Pose2d getStart() {
        switch(DriverStation.getRawAllianceStation()) {
          case Blue1: return new Pose2d(7.5, 6, Rotation2d.k180deg);
          case Blue2: return new Pose2d(7.5, 4, Rotation2d.k180deg);
          case Blue3: return new Pose2d(7.5, 2, Rotation2d.k180deg);

          case Red1: return new Pose2d(10, 6, Rotation2d.kZero);
          case Red2: return new Pose2d(10, 4, Rotation2d.kZero);
          default: return new Pose2d(10, 2, Rotation2d.kZero);
        }
    }
  }
}
