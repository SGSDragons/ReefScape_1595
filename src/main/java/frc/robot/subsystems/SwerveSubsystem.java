// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.Reefscape;
import frc.robot.LimelightHelpers.PoseEstimate;

import java.io.File;
import java.util.Arrays;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends DriveSubsystem {

    /**
     * Swerve drive object.
     */
    private final SwerveDrive swerveDrive;

    private final double maxSpeedMps;

    /**
     * Initialize {@link SwerveDrive} with the directory provided.
     *
     * @param maxSpeed The maximum operating speed to allow.
     */
    public SwerveSubsystem(LinearVelocity maxSpeed, Pose2d initialPose) {
        this.maxSpeedMps = maxSpeed.in(MetersPerSecond);

        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            File configs = new File(Filesystem.getDeployDirectory(), "swerve");
            SwerveParser configParser = new SwerveParser(configs);
            swerveDrive = configParser.createSwerveDrive(maxSpeedMps, initialPose);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        // Heading correction should only be used while controlling the robot via angle.
        swerveDrive.setHeadingCorrection(false);

        swerveDrive.setCosineCompensator(false);

        //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1);

        // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1);

        // Disable the odometry updates in a background thread and do them in
        // the main periodic loop. This way vision updates can be injected without
        // race conditions.
        swerveDrive.stopOdometryThread();
        setupPathPlanner();
    }

    @Override
    public void periodic() {
        swerveDrive.updateOdometry();
        PoseEstimate poseEst = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        if (poseEst != null && poseEst.tagCount > 0) {
            swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(
                poseEst.pose,
                poseEst.timestampSeconds,
                VecBuilder.fill(0.7, 0.7, 1e10)
                );
        }
    }

    @Override
    public void simulationPeriodic() {
    }

    /**
     * Setup AutoBuilder for PathPlanner.
     */
    public void setupPathPlanner() {
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
            final BiConsumer<ChassisSpeeds, DriveFeedforwards> driver;
            final boolean enableFeedForward = true;

            if (enableFeedForward) {
                driver = (speeds, feedForwards) -> swerveDrive.drive(speeds, swerveDrive.kinematics.toSwerveModuleStates(speeds), feedForwards.linearForces());
            } else {
                driver = (speeds, unused) -> swerveDrive.setChassisSpeeds(speeds);
            }

            // PPHolonomicController is the built in path following controller for holonomic drive trains
            final PathFollowingController controller = new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0),
                    // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0)
                    // Rotation PID constants
            );

            // Configure AutoBuilder last
            AutoBuilder.configure(
                    this::getPose,
                    this::resetOdometry,
                    this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    driver,
                    controller,
                    config,

                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    () -> Reefscape.isRedAlliance(),

                    // Reference to this subsystem to set requirements
                    this
            );

        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        //Preload PathPlanner Path finding
        // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
        PathfindingCommand.warmupCommand().schedule();
    }

    /**
     * Command to characterize the robot drive motors using SysId
     *
     * @return SysId Drive Command
     */
    public Command sysIdDriveMotorCommand() {
        SysIdRoutine routine = SwerveDriveTest.setDriveSysIdRoutine(new Config(), this, swerveDrive, 12, true);
        return SwerveDriveTest.generateSysIdCommand(routine, 3.0, 5.0, 3.0);
    }

    /**
     * Command to characterize the robot angle motors using SysId
     *
     * @return SysId Angle Command
     */
    public Command sysIdAngleMotorCommand() {
        SysIdRoutine routine = SwerveDriveTest.setAngleSysIdRoutine(new Config(), this, swerveDrive);
        return SwerveDriveTest.generateSysIdCommand(routine, 3.0, 5.0, 3.0);
    }

    /**
     * Returns a Command that centers the modules of the SwerveDrive subsystem.
     *
     * @return a Command that centers the modules of the SwerveDrive subsystem
     */
    public Command centerModulesCommand() {
        return run(() -> Arrays.asList(swerveDrive.getModules())
                .forEach(it -> it.setAngle(0.0)));
    }

    /**
     * Replaces the swerve module feedforward with a new SimpleMotorFeedforward object.
     *
     * @param kS the static gain of the feedforward
     * @param kV the velocity gain of the feedforward
     * @param kA the acceleration gain of the feedforward
     */
    public void replaceSwerveModuleFeedforward(double kS, double kV, double kA) {
        // TODO: We need to run the SysId routines to get these constants
        swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
    }

    /**
     * Command to drive the robot using translative values and heading as a setpoint.
     * Using this method, the robot will move in the direction of one joystick's tilt, and
     * point in the direction of the other joystick's tilt.
     *
     * @param translationX Translation in the X direction. Cubed for smoother controls.
     * @param translationY Translation in the Y direction. Cubed for smoother controls.
     * @param headingX     Heading X to calculate angle of the joystick.
     * @param headingY     Heading Y to calculate angle of the joystick.
     * @return Drive command.
     */
    @Override
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY, double scale) {
        swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
        return run(() -> {

            Translation2d joystick = new Translation2d(translationX.getAsDouble(), translationY.getAsDouble());
            double magnitude = joystick.getNorm();
          
            if (joystick.getNorm() < 0.1) {
                joystick = Translation2d.kZero;
            }


            magnitude = scale*Math.pow(magnitude, 3);
            joystick = joystick.times(magnitude);

            Translation2d scaledInputs = SwerveMath.scaleTranslation(joystick, 0.8);

            // Make the robot move
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(
                    scaledInputs.getX(),
                    scaledInputs.getY(),
                    headingX.getAsDouble(),
                    headingY.getAsDouble(),
                    swerveDrive.getOdometryHeading().getRadians(),
                    swerveDrive.getMaximumChassisVelocity()));
        });
    }

    @Override
    public Command driveRelative(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
    {
      return run(() -> {
        
        Translation2d joystick = new Translation2d(translationX.getAsDouble(), translationY.getAsDouble());
        if (joystick.getNorm() < 0.1) {
            joystick = Translation2d.kZero;
        }
        double omega = MathUtil.applyDeadband(angularRotationX.getAsDouble(), 0.2);
        omega = Math.pow(omega, 3) * swerveDrive.getMaximumChassisAngularVelocity() / 4.0;

        // Make the robot move
        swerveDrive.drive(joystick, omega, false, false);
      });
    }

    /**
     * Command to drive the robot using translative values and heading pointed to a fixed target
     *
     * @param translationX Translation in the X direction. Cubed for smoother controls.
     * @param translationY Translation in the Y direction. Cubed for smoother controls.
     * @return Drive command.
     */
    @Override
    public Command  driveTargeting(DoubleSupplier translationX, DoubleSupplier translationY) {
        swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
        return run(() -> {

            Translation2d joystick = new Translation2d(translationX.getAsDouble(), translationY.getAsDouble());

            if (joystick.getNorm() < 0.1) {
                joystick = Translation2d.kZero;
            }

            double heading = swerveDrive.getOdometryHeading().getDegrees();

            heading = Math.round(heading) * 60;
            

            Translation2d scaledInputs = SwerveMath.scaleTranslation(joystick, 0.8);
            Translation2d reef = Constants.Reefscape.getReefLocation();
            Translation2d direction = reef.minus(getPose().getTranslation());

            // Make the robot move
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(
                    scaledInputs.getX(),
                    scaledInputs.getY(),
                    direction.getY(),
                    direction.getX(),
                    swerveDrive.getOdometryHeading().getRadians(),
                    swerveDrive.getMaximumChassisVelocity()));
        });
    }

    /**
     * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
     * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
     * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
     *
     * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
     *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
     *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
     *                      (field North) and positive y is torwards the left wall when looking through the driver station
     *                      glass (field West).
     * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
     *                      relativity.
     * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation,
                rotation,
                fieldRelative,
                false); // Open loop is disabled since it shouldn't be used most of the time.
    }

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> swerveDrive.driveFieldOriented(velocity.get()));
    }

    /**
     * Drive according to the chassis robot oriented velocity.
     *
     * @param velocity Robot oriented {@link ChassisSpeeds}
     */
    public void drive(ChassisSpeeds velocity) {
        swerveDrive.drive(velocity);
    }


    /**
     * Get the swerve drive kinematics object.
     *
     * @return {@link SwerveDriveKinematics} of the swerve drive.
     */
    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
     * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
     * keep working.
     *
     * @param initialHolonomicPose The pose to set the odometry to
     */
    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
        resetController();
    }

    public void resetController() {
         swerveDrive.swerveController.lastAngleScalar = swerveDrive.getPose().getRotation().getRadians();
    }

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by odometry.
     *
     * @return The robot's pose
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Set chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds Chassis Speeds to set.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    /**
     * Post the trajectory to the field.
     *
     * @param trajectory The trajectory to post.
     */
    public void postTrajectory(Trajectory trajectory) {
        swerveDrive.postTrajectory(trajectory);
    }

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
     */
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    /**
     * This will zero (calibrate) the robot to assume the current position is facing forward
     * <p>
     * If red alliance rotate the robot 180 after the drivebase zero command
     */
    public void zeroGyroWithAlliance() {
        zeroGyro();
        if (Constants.Reefscape.isRedAlliance()) {
            //Set the pose 180 degrees
            resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
        }
    }

    /**
     * Sets the drive motors to brake/coast mode.
     *
     * @param brake True to set motors to brake mode, false for coast.
     */
    public void setMotorBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    /**
     * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
     * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
     *
     * @return The yaw angle
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
     * the angle of the robot.
     *
     * @param xInput   X joystick input for the robot to move in the X direction.
     * @param yInput   Y joystick input for the robot to move in the Y direction.
     * @param headingX X joystick which controls the angle of the robot.
     * @param headingY Y joystick which controls the angle of the robot.
     * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
        return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                scaledInputs.getY(),
                headingX,
                headingY,
                getHeading().getRadians(),
                maxSpeedMps);
    }

    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current field-relative velocity
     */
    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    /**
     * Gets the current velocity (x, y and omega) of the robot
     *
     * @return A {@link ChassisSpeeds} object of the current velocity
     */
    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * Get the {@link SwerveController} in the swerve drive.
     *
     * @return {@link SwerveController} from the {@link SwerveDrive}.
     */
    public SwerveController getSwerveController() {
        return swerveDrive.swerveController;
    }

    /**
     * Get the {@link SwerveDriveConfiguration} object.
     *
     * @return The {@link SwerveDriveConfiguration} fpr the current drive.
     */
    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return swerveDrive.swerveDriveConfiguration;
    }

    /**
     * Lock the swerve drive to prevent it from moving.
     */
    public void lock() {
        swerveDrive.lockPose();
    }

    /**
     * Gets the current pitch angle of the robot, as reported by the imu.
     *
     * @return The heading as a {@link Rotation2d} angle
     */
    public Rotation2d getPitch() {
        return swerveDrive.getPitch();
    }

    /**
     * Gets the swerve drive object.
     *
     * @return {@link SwerveDrive}
     */
    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }
}