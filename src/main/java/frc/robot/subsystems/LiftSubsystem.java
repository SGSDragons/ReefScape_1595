package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
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

public class LiftSubsystem extends SubsystemBase{

    TalonFX rightLiftMotor;
    TalonFX leftLiftMotor;
    PIDController pid = new PIDController(0, 0, 0);


    public LiftSubsystem() {
        
        rightLiftMotor = new TalonFX(LiftConstants.rightLiftMotorCanId);
        leftLiftMotor = new TalonFX(LiftConstants.leftLiftMotorCanId);

        leftLiftMotor.setControl(new Follower(LiftConstants.rightLiftMotorCanId, true));

        rightLiftMotor.setNeutralMode(NeutralModeValue.Brake);
        
    }

    public void stopLift() {
        rightLiftMotor.set(0.0);
    }

    public void LiftControl(double power) {
        rightLiftMotor.set(power);
    }


    public enum LiftPosition {
        LOWERED(0.0),
        SHELF(1.0),
        LOWPOST(2),
        HIGHPOST(3);

        double setPoint;
        LiftPosition(double setPoint) {
            this.setPoint = setPoint;
        }
    }

    public Command gotoPosition(LiftPosition position) {

        var config = new Slot0Configs();
        config.kP = LiftConstants.kP;
        config.kI = LiftConstants.kI;
        config.kD = LiftConstants.kD;

        rightLiftMotor.getConfigurator().apply(config);

        return run(() -> {

            // create a position closed-loop request, voltage output, slot 0 configs
            final PositionVoltage lift_request = new PositionVoltage(0).withSlot(0);

            // set position to 10 rotations
            rightLiftMotor.setControl(lift_request.withPosition(position.setPoint));
        });
    }

    public Command move(DoubleSupplier axis) {

        return run(() -> {
            double offset = axis.getAsDouble();
            if (Math.abs(offset) < 0.2) {
                offset = 0.0;
            }

            final VelocityVoltage leftController = new VelocityVoltage(offset);
            final VelocityVoltage rightController = new VelocityVoltage(offset);
            rightLiftMotor.set(offset);

            // rightLiftMotor.setControl(rightController);
            // rightLiftMotor.getMotorVoltage();
            // leftLiftMotor.setControl(leftController);
        });

    }
    
    @Override
    public void periodic() {
        telemetry();
    }

    public void telemetry() {
        // SmartDashboard.putNumber("Left Motor Velocity", leftLiftMotor.getVelocity().getValueAsDouble());
        // SmartDashboard.putNumber("Left Motor Position", leftLiftMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Right Motor Velocity", rightLiftMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Right Motor Position", rightLiftMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Right Motor Voltage", rightLiftMotor.getMotorVoltage().getValueAsDouble());
    }
}
