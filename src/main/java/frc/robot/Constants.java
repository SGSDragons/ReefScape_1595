// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final double stickDeadband() { return Preferences.getDouble("Deadband", 0.1); }

    // Austin controller button names --> values
    public static final int backLeftSingleSwitch = 1;
    public static final int topLeftToggleBack = 2;
    public static final int topLeftToggleForward = 3;
    public static final int frontFarLeftToggleUp = 4;
    public static final int frontFarLeftToggleDown = 5;
    public static final int frontNearLeftToggleUp = 6;
    public static final int frontNearLeftToggleDown = 7;
    public static final int frontRightToggleDown = 8; 
    public static final int frontRightToggleUp = 9;
    public static final int topRightToggleForward = 10;
    public static final int topRightToggleBack = 11;
    public static final int backRightSingleSwitch = 12;
    public static final int topLeftButton = 13; 
    public static final int resetButton = 14;
    public static final int cancelButton = 15;
    public static final int rollerButton = 16;
    public static final int rollerLeft = 17;
    public static final int rollerRight = 18;
    public static final int frontLeftBottomSwitchLeft = 19;
    public static final int frontLeftBottomSwitchRight = 20;
    public static final int frontLeftMiddleSwitchDown = 21;
    public static final int frontLeftMiddleSwitchUp = 22;
    public static final int frontRightBottomSwitchLeft = 23;
    public static final int frontRightBottomSwitchRight = 24;
    public static final int frontRightMiddleSwitchDown = 25;  
    public static final int frontRightMiddleSwitchUp = 26;
}

  //Hardware IDs for parts on robot (excluding drivetrain).
  public static final class HardwareID {
    public static final int leftClimberForwardChannel = 4;
    public static final int leftClimberReverseChannel = 5;
    public static final int rightClimberForwardChannel = 6;
    public static final int rightClimberReverseChannel = 7;
    public static final int leftNoteAimerForwardChannel = 0;
    public static final int leftNoteAimerReverseChannel = 1;
    public static final int rightNoteAimerForwardChannel = 2;
    public static final int rightNoteAimerReverseChannel = 3;

    public static final int indexerMotorCANId = 18;
    public static final int indexerMotor2CANId = 15;
    public static final int bottomSpinnerMotorCANId = 19;
    public static final int middleSpinnerMotorCANId = 17;
    public static final int topSpinnerMotorCANId = 16;
  }

  public static final class TuningValues {
    public static final double launcherkV = 0.14;
    public static final double launcherkP = 8.0;
    public static final double launcherkI = 0.001;
    public static final double launcherkD = 0.0;
  }

  public static final class SystemToggles {
    public static final boolean useCompleteAuto = false;
  }

    public static final class AutoConstants { //TO BE TUNED
         public static final double kMaxSpeedMetersPerSecond = 3;
         public static final double kMaxAccelerationMetersPerSecondSquared = 3;
         public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
         public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
     
         /*
          public static final double kPXController = 3;
          public static final double kPYController = 3;
          public static final double kPThetaController = 4;
          */

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

      // Intake, Index, Launch
      public static final String indexVoltKey = "Indexer Voltage";
      public static final String indexAmpVoltKey = "Indexer Amp-Shot Voltage";
      public static final String intakeVoltKey = "Intake Voltage";
      public static final String speakerHighAimV = "Speaker-Shot High-Aim Velocity";
      public static final String speakerLowAimV = "Speaker-Shot Lower-Aim Velocity";
      public static final String ampV = "Amplifier-Shot Velocity";
      public static final String launcherTolerance = "Launcher Tolerance";

      public static final String characterizationKey = "System Characterization Mode";
      public static final String compressorOnlyKey = "Compressor Only Mode";

      public static final String correctNotePositionTimeKey = "Note Correction Outtake Seconds";
      public static final String minimumNoteProximityKey = "Note Detection Proximity";

    }

    public class LiftConstants{
      public static final double kS = 0.5;
      public static final double kG = 0;
      public static final double kP = 0;
      public static final double kI = 0;
      public static final double kD = 0;

      public static final int rightLiftMotorCanId = 16;
      public static final int leftLiftMotorCanId = 0;

      public static final double Lowered = 0;
      public static final double Shelf = 13;
      public static final double Low = 17;
      public static final double Medium = 25;
      public static final double High = 35;

      public double RotationstoInches() {
        return 0;
      }
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
