package frc.robot.subsystems;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.I2C;
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.Preferences;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.compound.Diff_DutyCycleOut_Velocity;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.HardwareID;
// import frc.lib.utilities.LimelightHelpers;
// import frc.lib.utilities.Constants.HardwareID;
// import frc.lib.utilities.Constants.Keys;
// import frc.robot.Constants.Keys;
import frc.robot.Constants.ClimbConstants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;

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

import java.util.function.DoubleSupplier;
// import com.revrobotics.ColorSensorV3;

public class ClimbSubsystem extends SubsystemBase{

    TalonFX climbMotor;

    private boolean hasExtended = false;
    private double extendedPosition = 1.0;
    private double retractedPosition = 2.0;

    public ClimbSubsystem() {
        
        climbMotor = new TalonFX(ClimbConstants.ClimberMotorcanId);
        climbMotor.setNeutralMode(NeutralModeValue.Brake);
        
    }

    public void activate() {
        final double targetPosition;
        if (hasExtended == false) {
            targetPosition = extendedPosition;
        } else {
            targetPosition = retractedPosition;
        }

        PositionVoltage target = new PositionVoltage(targetPosition);
        climbMotor.setControl(target);
    }

    public Command climbStop() {
        return run(() -> climbMotor.set(0.0));
    }

    public void climbUp() {
        final PositionVoltage position = new PositionVoltage(0).withSlot(0);
        climbMotor.setControl(position);
    }

    public void climbDown() {
        final PositionVoltage position = new PositionVoltage(0).withSlot(0);
        climbMotor.setControl(position);
    }

    public boolean hasExtended() {
        return hasExtended;
    }

    public Command drive(DoubleSupplier speedSupplier) {
        return run(() -> {
            double speed = speedSupplier.getAsDouble();
            if (Math.abs(speed) < 0.1) {
                speed = 0.0;
            }

            climbMotor.set(speed);
            // climbMotor.setControl(new VelocityVoltage(speed));
        });
    }
    public Command reconfigure() {
        return runOnce(() -> {
            var config = new Slot0Configs();
            config.kP = getPreference("     ", 0.0);
            climbMotor.getConfigurator().apply(config);
        });
    }

    @Override
    public void periodic() {
        if (climbMotor.getPosition().getValueAsDouble() > 0.9 * extendedPosition) {
            hasExtended = true;
        }

        telemetry();
    }

    public void telemetry() {
        SmartDashboard.putNumber("Climb Motor Velocity", climbMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Climb Motor Position", climbMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Climb Motor Voltage", climbMotor.getMotorVoltage().getValueAsDouble());
    }

    public double getPreference(String key, double fallback) {
        return Preferences.getDouble("climb/" + key, fallback);
    }
}
