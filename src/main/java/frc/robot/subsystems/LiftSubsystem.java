package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class LiftSubsystem extends SubsystemBase{

    final TalonFX motor;
    final BooleanSupplier bottomReached;

    public LiftSubsystem() {
        
        TalonFX rightLiftMotor = new TalonFX(LiftConstants.rightLiftMotorCanId);
        TalonFX leftLiftMotor = new TalonFX(LiftConstants.leftLiftMotorCanId);

        leftLiftMotor.setControl(new Follower(LiftConstants.rightLiftMotorCanId, true));
        rightLiftMotor.setNeutralMode(NeutralModeValue.Brake);
        motor = rightLiftMotor;

        // CB: When we wire the limit switch
        //bottomReached = new DigitalInput(0)::get;
        bottomReached = () -> false;

        reconfigurePid();
    }

    public void reconfigurePid() {
        final var config = new Slot0Configs();
        config.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        config.kG = getPreference("gravity", LiftConstants.kG);
        config.kS = getPreference("static", LiftConstants.kS);
        config.kP = getPreference("proportional", LiftConstants.kP);
        config.kI = getPreference("integral", LiftConstants.kI);
        config.kD = getPreference("derivative", LiftConstants.kD);

        motor.getConfigurator().apply(config);
    }


    public void stopLift() {
        motor.set(0.0);
    }

    public static class LiftPosition {
        
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

    public Command goToGround() {

        return run(() -> {
            if (motor.getPosition().getValueAsDouble() > 0.2) {
                // Drive with a PID when far away for max speed.
                motor.setControl(new PositionVoltage(0));
            } else if (!bottomReached.getAsBoolean()) {
                // Drive slowly until the limit switch is not reached
                motor.set(-0.1);
            } else {
                // It's at the bottom. Stop the motor and rezero the encoder
                motor.set(0.0);
                motor.setPosition(0.0);
            }
        });
    }

     public Command move(DoubleSupplier axis) {
         return run(() -> {
             double speed = MathUtil.applyDeadband(axis.getAsDouble(), 0.2);
             motor.set(speed);
         });
    }
    
    @Override
    public void periodic() {
        telemetry();
    }


    public void telemetry() {
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
