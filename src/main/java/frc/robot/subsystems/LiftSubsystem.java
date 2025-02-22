package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Preferences;
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

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.revrobotics.ColorSensorV3;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class LiftSubsystem extends SubsystemBase{

    final TalonFX motor;

    public LiftSubsystem() {
        
        TalonFX rightLiftMotor = new TalonFX(LiftConstants.rightLiftMotorCanId);
        TalonFX leftLiftMotor = new TalonFX(LiftConstants.leftLiftMotorCanId);

        leftLiftMotor.setControl(new Follower(LiftConstants.rightLiftMotorCanId, true));
        rightLiftMotor.setNeutralMode(NeutralModeValue.Brake);

        var config = new Slot0Configs();
        config.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        config.kG = getPreference("gravity", LiftConstants.kG);
        config.kS = getPreference("static", LiftConstants.kS);
        config.kP = getPreference("proportional", LiftConstants.kP);
        config.kI = getPreference("integral", LiftConstants.kI);
        config.kD = getPreference("derivative", LiftConstants.kD);

        rightLiftMotor.getConfigurator().apply(config);

        motor = rightLiftMotor;
    }


    public void stopLift() {
        motor.set(0.0);
    }

    public void LiftControl(double power) {
        motor.set(power);
    }


    public class LiftPosition {
        
        double setPoint;
        String name;

        public LiftPosition(String name, double fallback) {
            this.setPoint = getPreference(name, fallback);
            this.name = name;
        }

        public void adjust(double shift) {
            this.setPoint += shift;
            setPreference(name, shift);
        }
    }

    public final LiftPosition Ground = new LiftPosition("Ground", LiftConstants.Ground); // Should be 0
    public final LiftPosition Shelf = new LiftPosition("Shelf", LiftConstants.Shelf);
    public final LiftPosition Low  = new LiftPosition("Low", LiftConstants.Low);
    public final LiftPosition Medium = new LiftPosition("Medium", LiftConstants.Medium);
    public final LiftPosition High = new LiftPosition("High", LiftConstants.High);

    public Command gotoPosition(LiftPosition position, DoubleSupplier axis) {


        return run(() -> {

            double offset = axis.getAsDouble();
            if (Math.abs(offset) > 0.2) {
                position.adjust(offset * 0.01);
            }

            // create a position closed-loop request, voltage output, slot 0 configs
            final PositionVoltage lift_request = new PositionVoltage(position.setPoint).withSlot(0);

            // set position to 10 rotations
            motor.setControl(lift_request);
        });
    }

    public Command Ground() {

        return run(() -> {

            final PositionVoltage lift_request = new PositionVoltage(0).withSlot(0);
            motor.setControl(lift_request); 

        });

    }

    // public Command move(DoubleSupplier axis) {

    //     return run(() -> {
    //         double offset = axis.getAsDouble();
    //         if (Math.abs(offset) < 0.2) {
    //             offset = 0.0;
    //         }

    //         final VelocityVoltage leftController = new VelocityVoltage(offset);
    //         final VelocityVoltage rightController = new VelocityVoltage(offset);
    //         //rightLiftMotor.set(offset);

    //         // create a position closed-loop request, voltage output, slot 0 configs
    //         final PositionVoltage lift_request = new PositionVoltage(0).withSlot(0);

    //         // set position to 10 rotations
    //         rightLiftMotor.setControl(lift_request.withPosition(position.setPoint));

    //         // rightLiftMotor.setControl(rightController);
    //         // rightLiftMotor.getMotorVoltage();
    //         // leftLiftMotor.setControl(leftController);
    //     });

    // }
    
    @Override
    public void periodic() {
        telemetry();
    }


    public void telemetry() {
        // SmartDashboard.putNumber("Left Motor Velocity", leftLiftMotor.getVelocity().getValueAsDouble());
        // SmartDashboard.putNumber("Left Motor Position", leftLiftMotor.getPosition().getValueAsDouble());

        SmartDashboard.putNumber("Right Motor Velocity", motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Right Motor Position", motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Right Motor Voltage", motor.getMotorVoltage().getValueAsDouble());
    }

    private static double getPreference(String key, double fallback) {
        return Preferences.getDouble("Lift/" + key, fallback);
    }

    private static void setPreference(String key, double newValue) {
        Preferences.setDouble("Lift/" + key, newValue);
    }
}
