package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.util.Arrays;
import java.util.List;

public class ApproachReef extends Command {

    private final SwerveSubsystem swerve;

    private final PathConstraints constraints = new PathConstraints(
            8.0,
            12.0,
            540.0,
            720.0,
            12.0,
            false
    );

    private Command activePath;

    public ApproachReef(SwerveSubsystem drive) {
        this.swerve = drive;
    }

    @Override
    public void schedule() {
        super.schedule();
    }

    @Override
    public void initialize() {
        super.initialize();
        ChassisSpeeds chassisSpeeds = swerve.getFieldVelocity();
        Pose2d currentPose = swerve.getPose();
        Translation2d velocity = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);

        Approach approach = Approach.best(currentPose.getTranslation(), velocity, Constants.Reefscape.getReefLocation());
        Translation2d reef = Constants.Reefscape.getReefLocation();

        List<Pose2d> poses = Arrays.asList(
            swerve.getPose(),
            new Pose2d(reef.plus(approach.reefOffset), approach.robotHeading),
            new Pose2d(reef.plus(approach.reefEdge), approach.robotHeading)
        );
        if (SwerveDriveTelemetry.isSimulation) {
            swerve.getSwerveDrive().field.getObject("Trajectory").setPoses(poses);
        }

        // The list of states the robot should transition through.
        // First will be the robot's initial state (position and velocity)
        // Second will be
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                new IdealStartingState(velocity.getNorm(), currentPose.getRotation()),
                new GoalEndState(0.0, approach.robotHeading),
                false);
        path.preventFlipping = true;

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
        swerve.resetController();
    }

    @Override
    public boolean isFinished() {
        return activePath.isFinished();
    }

    // Selects the best approach angle for the reef based on the robot's
    // current position and velocity.
    private static final int[] boundaries = new int[] { -150, -90, -30, 30, 90, 150 };
    enum Approach {
        LEFT(-180),
        LOWER_LEFT(-120),
        LOWER_RIGHT(-60),
        RIGHT(0),
        UPPER_RIGHT(60),
        UPPER_LEFT(120);

        double beginAngle;
        double midAngle;
        double endAngle;
        final Translation2d reefEdge;
        final Translation2d reefOffset;
        final Rotation2d robotHeading;

        private static final List<Approach> all = Arrays.asList(Approach.values());

        // Find the best approach to use based on the robot's current state and its target reef.
        public static Approach best(Translation2d location, Translation2d velocity, Translation2d reefLocation) {
            Translation2d estimatedLocation = location.plus(velocity.times(0.25));
            Translation2d reefRelative = estimatedLocation.minus(reefLocation);

            final double angle = Math.atan2(reefRelative.getY(), reefRelative.getX());

            for (Approach approach : all) {
                if (approach.beginAngle <= angle && angle < approach.endAngle) {
                    return approach;
                }
            }

            // Assume it's the left, because that gets weird with the [-210, -150] and atan.
            return Approach.LEFT;
        }

        Approach(double angle) {
            beginAngle = Math.toRadians(angle - 30.0);
            midAngle = Math.toRadians(angle);
            endAngle = Math.toRadians(angle + 30.0);


            // The reef's wall is exactly 1m from the center of the reef, and units are in Meters
            reefEdge = new Translation2d(Math.cos(midAngle), Math.sin(midAngle));

            // Set the path's intermediate point a half meter out for a clean approach
            reefOffset = reefEdge.times(1.5);

            // Make the robot point towards the reef, instead of away from the reef
            robotHeading = Rotation2d.fromRadians(midAngle + Math.PI);
        }
    }
}
