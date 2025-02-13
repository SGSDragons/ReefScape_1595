package frc.robot.subsystems;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.I2C;
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.HardwareID;
// import frc.lib.utilities.LimelightHelpers;
// import frc.lib.utilities.Constants.HardwareID;
// import frc.lib.utilities.Constants.Keys;
// import frc.robot.Constants.Keys;
import frc.robot.Constants.LiftConstants;

// import java.util.function.DoubleSupplier;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix6.configs.MotorOutputConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.revrobotics.ColorSensorV3;

public class ClimbSubsystem extends SubsystemBase{

    TalonFX ClimbMotor;


    public ClimbSubsystem() {
        
        ClimbMotor = new TalonFX(LiftConstants.ClimberMotorcanId);
        ClimbMotor.setNeutralMode(NeutralModeValue.Brake);
        
    }

    public Command stopClimb() {
        return run(() -> { ClimbMotor.set(0.0); });
    }

    public Command Climbup() {
        return run(() -> { ClimbMotor.set(0.5); });
    }

    public Command Climbdown() {
        return run(() -> { ClimbMotor.set(-0.5); });
    }

    
    @Override
    public void periodic() {
        telemetry();
    }

    public void telemetry() {
        SmartDashboard.putNumber("Climb Motor Velocity", ClimbMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Climb Motor Position", ClimbMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Climb Motor Voltage", ClimbMotor.getMotorVoltage().getValueAsDouble());
    }
}
