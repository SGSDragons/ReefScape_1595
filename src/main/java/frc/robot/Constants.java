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

      public static final int rightLiftMotorCanId = 16; //Lift motors
      public static final int leftLiftMotorCanId = 0; //Lift motors

      public static final int ClimberMotorcanId = 15; //Climber motor

      public static final int intakeRotationMotorCANId = 1; //Coral intake motors
      public static final int frontWheelsMotorCANId = 2; //Coral intake motors
      public static final int sideWheelsMotorCANId = 3; //Coral intake motors
  }

  public static final class SystemToggles {
    public static final boolean useCompleteAuto = false;
  }

    public static final class AutoConstants { //TO BE TUNED
         public static final double kMaxSpeedMetersPerSecond = 3;
         public static final double kMaxAccelerationMetersPerSecondSquared = 3;
         public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
         public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

          public static double kPXController = Preferences.getDouble(Keys.auto_kPXKey, 3);
          public static double kPThetaController = Preferences.getDouble(Keys.auto_kPThetaKey, 4);
     
         /* Constraint for the motion profilied robot angle controller */
         public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
             new TrapezoidProfile.Constraints(
                 kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

     }

     public static final class Keys {
      // Driving
      public static final String angle_kPKey = "Angle kP";
      public static final String drive_kPKey = "Drive kP";
      public static final String drive_kSKey = "Drive kS";
      public static final String drive_kVKey = "Drive kV";
      public static final String drive_kAKey = "Drive kA";
      public static final String auto_kPXKey = "Auto kP X";
      public static final String auto_kPThetaKey = "Auto kP Theta";
      public static final String maxSpeedKey = "Max Speed";
      public static final String maxAngularVelocityKey = "Max Angular Velocity";
    }

    public class LiftConstants{

      public static final double kS = 0.5;
      public static final double kG = 0;
      public static final double kP = 0;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kGravity = 0;

      public static final int rightLiftMotorCanId = 16;
      public static final int leftLiftMotorCanId = 0;

      public static final double Ground = 0;
      public static final double Shelf = 13;
      public static final double Low = 17;
      public static final double Medium = 25;
      public static final double High = 35;

    }

    public class CarriageConstants {

      public static final int rotationMotorCanId = 1;
      public static final int directionChannel = 0;
      public static final int coralMotorCanId = 0;

      public static final double TopAngle = -0.5;
      public static final double DefaultAngle = 0;
      public static final double intakeSpeed = 0.5;

    }

    public class CoralIntakeConstants {

      public static final int intakeRotationMotorCanId = 0;
      public static final int frontWheelsMotorCanId = 0;
      public static final int sideWheelsMotorCanId = 0;

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

    public class ClimbConstants {
      public static final int ClimberMotorcanId = 15;
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
