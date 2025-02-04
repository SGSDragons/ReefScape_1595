package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ApproachFactory;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.util.List;

// A command that can be bound to a trigger.
// Whenever scheduled, it dynamically generates a new path based on the
// robot's state and follows it.
public class DynamicReefApproach extends Command {

    private final SwerveSubsystem swerve;
    private final ApproachFactory factory;
    private Command activePath;

    public DynamicReefApproach(SwerveSubsystem drive, ApproachFactory factory) {
        this.swerve = drive;
        this.factory = factory;
    }

    @Override
    public void initialize() {
        super.initialize();
        final ChassisSpeeds chassisSpeeds = swerve.getFieldVelocity();
        final Translation2d location = swerve.getPose().getTranslation();
        final Translation2d velocity = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);

        final PathPlannerPath path = factory.generateApproach(location, velocity);

        if (SwerveDriveTelemetry.isSimulation) {
            List<Pose2d> poses = path.getAllPathPoints().stream()
                    .map(p -> new Pose2d(p.position, Rotation2d.kZero))
                    .toList();
            swerve.getSwerveDrive().field.getObject("Trajectory").setPoses(poses);
        }

        activePath = AutoBuilder.followPath(path);
        activePath.initialize();
    }

    @Override
    public void execute() {
        activePath.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        activePath.end(interrupted);

        // ensure the controller's target heading matches the robot's current heading
        swerve.resetController();
    }

    @Override
    public boolean isFinished() {
        return activePath.isFinished();
    }
}
