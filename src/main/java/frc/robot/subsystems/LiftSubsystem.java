package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utilities.LimelightHelpers;
import frc.lib.utilities.Constants.HardwareID;
import frc.lib.utilities.Constants.Keys;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.ColorSensorV3;

public class LiftSubsystem extends SubsystemBase{

    TalonFX LiftMotor;
    //ColorSensorV3 colorSensorLeft, colorSensorRight;

    public LiftSubsystem() {
        
        LiftMotor.setNeutralMode(NeutralModeValue.Brake);
        LiftMotor.setInverted(true);
        
    }

    public void stopLift() {
        LiftMotor.setVoltage(0.0);
    }

    public void indexNoteIntake() {
        if (!noteLoaded) {
            LiftMotor.setVoltage(Preferences.getDouble(Keys.intakeVoltKey, 3.0));
            return;
        }

        LiftMotor.setVoltage(0.0);
    }

    public void indexNoteOuttake() {
        LiftMotor.setVoltage(-Preferences.getDouble(Keys.intakeVoltKey, 3.0));
    }

    public void indexNoteLaunchSpeaker(boolean ampShot) {
        double voltage = ampShot ? Preferences.getDouble(Keys.indexAmpVoltKey, 3.0) : Preferences.getDouble(Keys.indexVoltKey, 6.0);
        LiftMotor.setVoltage(voltage);
    }

    public void indexNoteIntakeDisregardLoading() {
        LiftMotor.setVoltage(Preferences.getDouble(Keys.intakeVoltKey, 3.0));
    }
    
    @Override
    public void periodic() {

        int noteProximity = noteDetector.getProximity();
        if (noteProximity > Preferences.getDouble(Keys.minimumNoteProximityKey, 500)) {
            LimelightHelpers.setLEDMode_ForceBlink("limelight");
            noteLoaded = true;
        }
        else if (DriverStation.isTeleop()){
            LimelightHelpers.setLEDMode_ForceOff("limelight");
        }
        
        /*
        int leftProximity = colorSensorLeft.getProximity();
        int rightProximity = colorSensorRight.getProximity();
        if (leftProximity > 250 || rightProximity > 250) {
            noteLoaded = true;
        }
        else {
            noteLoaded = false;
        }
        */

        telemetry();
    }

    public void telemetry() {
        SmartDashboard.putNumber("Indexer Motor Velocity", LiftMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake Motor Velocity", intakeMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putBoolean("Note Loaded", noteLoaded);
        SmartDashboard.putNumber("Note Proximity", noteDetector.getProximity());
    }
}
