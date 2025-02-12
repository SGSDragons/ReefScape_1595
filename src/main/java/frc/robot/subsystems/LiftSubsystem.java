package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.lib.utilities.LimelightHelpers;
// import frc.lib.utilities.Constants.HardwareID;
// import frc.lib.utilities.Constants.Keys;
import frc.robot.Constants.Keys;
import frc.robot.Constants.LiftConstants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.ColorSensorV3;

public class LiftSubsystem extends SubsystemBase{

    TalonFX LiftMotor;
    PIDController pid = new PIDController(0, 0, 0);;

    public LiftSubsystem() {
        
        LiftMotor.setNeutralMode(NeutralModeValue.Brake);
        // LiftMotor.setInverted(true);
        
    }

    public void stopLift() {
        LiftMotor.setVoltage(0.0);
    }

    public void Liftup(double Joystick) {
        // LiftMotor.setVoltage(Preferences.getDouble(Keys.intakeVoltKey, 3.0));
        LiftMotor.setVoltage(Joystick);
    }

    public void Liftdown(double Joystick) {
        // LiftMotor.setVoltage(-Preferences.getDouble(Keys.intakeVoltKey, 3.0));
        LiftMotor.setVoltage(Joystick);
    }

    public void Ground() {

    }

    public void Level1() {

    }

    public void Level2() {
        
    }

    public void Level3() {
        
    }

    public void Level4() {
        
    }
    
    @Override
    public void periodic() {

        // int noteProximity = noteDetector.getProximity();
        // if (noteProximity > Preferences.getDouble(Keys.minimumNoteProximityKey, 500)) {
        //     LimelightHelpers.setLEDMode_ForceBlink("limelight");
        //     noteLoaded = true;
        // }
        // else if (DriverStation.isTeleop()){
        //     LimelightHelpers.setLEDMode_ForceOff("limelight");
        // }
        
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
        SmartDashboard.putNumber("Lift Motor Velocity", LiftMotor.getVelocity().getValueAsDouble());
    }
}
