package frc.robot.swerve;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotation;

public class SwerveModule {
    private static final CTREConfigs ctreConfigs = new CTREConfigs();

    public int moduleNumber;

    /**
     * The angular offset of the can coder where 0 points the wheel forward
     *
     * TODO: Use the magnet offset on the can coder directly
     * @see com.ctre.phoenix6.configs.MagnetSensorConfigs
     */
    private final TalonFX angleMotor;
    private final TalonFX driveMotor;
    private final CANcoder angleEncoder;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(SwerveConstants.driveKS, SwerveConstants.driveKV, SwerveConstants.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    public SwerveModule(int moduleNumber, SwerveConstants.Module moduleConstants){
        this.moduleNumber = moduleNumber;

        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.canCoderID());
        ctreConfigs.swerveCANcoderConfig.MagnetSensor.withMagnetOffset(moduleConstants.canOffset());
        angleEncoder.getConfigurator().apply(ctreConfigs.swerveCANcoderConfig);

        /* Angle Motor Config */
        angleMotor = new TalonFX(moduleConstants.angleMotorID());
        angleMotor.getConfigurator().apply(ctreConfigs.swerveAngleFXConfig);
        angleMotor.setPosition(angleEncoder.getPosition().getValue());

        /* Drive Motor Config */
        driveMotor = new TalonFX(moduleConstants.driveMotorID());
        driveMotor.getConfigurator().apply(ctreConfigs.swerveDriveFXConfig);
        driveMotor.getConfigurator().setPosition(0.0);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState.optimize(getState().angle);

        angleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));

        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / SwerveConstants.maxSpeed();
            driveMotor.setControl(driveDutyCycle);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, SwerveConstants.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            driveMotor.setControl(driveVelocity);
        }
    }

    public Rotation2d getAngle() {
        return new Rotation2d(angleEncoder.getPosition().getValue());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(driveMotor.getVelocity().getValueAsDouble(), SwerveConstants.wheelCircumference),
            getAngle()
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(driveMotor.getPosition().getValueAsDouble(), SwerveConstants.wheelCircumference),
            getAngle()
        );
    }

    public void setDriveVoltage(final Voltage volts) {
        driveMotor.setVoltage(volts.in(Units.Volts));

    }

    public void logStatus(final SysIdRoutineLog log) {
        double rotations = SwerveConstants.driveGearRatio * driveMotor.getPosition().getValueAsDouble();
        double linearDistance = rotations * SwerveConstants.wheelCircumference;

        log.motor(Integer.toString(moduleNumber))
                .voltage(driveMotor.getMotorVoltage().getValue())
                .linearPosition(Meters.of(linearDistance))
                .angularVelocity(driveMotor.getVelocity().getValue());
    }

    public void recalibrateCANCoder() {
        Preferences.setDouble(preference("angle"), angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    private String preference(final String id) {
        return "CANCoder_" + moduleNumber + "/" + id;

    }
}