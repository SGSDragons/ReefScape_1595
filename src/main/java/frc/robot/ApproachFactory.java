package frc.robot;

import com.pathplanner.lib.path.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.List;

// A command factory for making approach paths to the reef based on current telemetry
public class ApproachFactory {
    private final Translation2d reef;
    public final List<Approach> approaches;

    public final PathConstraints constraints = new PathConstraints(
            8.0,
            24.0,
            540.0,
            720.0,
            12.0,
            false
    );

    public ApproachFactory(Translation2d reef) {
        this.reef = reef;
        this.approaches = List.of(
                new Approach(-180),
                new Approach(-120),
                new Approach(-60),
                new Approach(0),
                new Approach(60),
                new Approach(120)
        );
    }

    public PathPlannerPath generateApproach(Translation2d location, Translation2d velocity) {
        final Approach best = bestApproach(location, velocity);
        return best.generatePath(location, velocity);
    }

    public Approach forAngleDegrees(double degrees) {
        return forAngle(Math.toRadians(degrees));
    }

    private Approach forAngle(double angle) {
        while (angle < approaches.get(0).beginAngle) {
            angle += 2*Math.PI;
        }
        while (angle >= approaches.get(approaches.size()-1).endAngle) {
            angle -= 2*Math.PI;
        }

        for (Approach approach : approaches) {
            if (approach.beginAngle <= angle && angle < approach.endAngle) {
                return approach;
            }
        }
        // Unreachable
        return approaches.get(0);
    }

    // Find the best approach to use based on the robot's current state and its target reef.
    public Approach bestApproach(Translation2d location, Translation2d velocity) {
        Translation2d estimatedLocation = location.plus(velocity.times(0.5));
        Translation2d reefRelative = estimatedLocation.minus(reef);
        return forAngle(Math.atan2(reefRelative.getY(), reefRelative.getX()));
    }

    // Precomputed approach data towards a reef at a specific angle.
    public class Approach {
        public final double beginAngle;
        public final double endAngle;
        public final Translation2d entryControl;
        public final Translation2d reefEdge;
        public final Translation2d reefOffset;
        public final Rotation2d robotHeading;
        public final RotationTarget rotation;

        public Approach(double angle) {
            double midAngle = Math.toRadians(angle);
            beginAngle = Math.toRadians(angle - 30.0);
            endAngle = Math.toRadians(angle + 30.0);


            // The reef's wall is exactly 1m from the center of the reef, and units are in Meters
            Translation2d relativeWall = new Translation2d(Math.cos(midAngle), Math.sin(midAngle));

            // The key points in field space.
            reefEdge = reef.plus(relativeWall.times(1.5));
            reefOffset = reef.plus(relativeWall.times(2.0));
            entryControl = reef.plus(relativeWall.times(2.5));

            // Make the robot point towards the reef, instead of away from the reef
            robotHeading = Rotation2d.fromRadians(midAngle + Math.PI);
            rotation = new RotationTarget(1, robotHeading);
        }

        // Generates a path for this approach for the robot's initial location and velocity
        public PathPlannerPath generatePath(Translation2d location, Translation2d velocity) {

            // The spline the robot should follow.
            // The first point is accounts for the robots current position and immediate trajectory
            // The second and third waypoints are to ensure a direct approach
            final List<Waypoint> waypoints = List.of(
                    new Waypoint(null, location, location.plus(velocity.times(0.1))),
                    new Waypoint(entryControl, reefOffset, reefEdge),
                    new Waypoint(reefOffset, reefEdge, null)
            );

            final PathPlannerPath path = new PathPlannerPath(
                    waypoints,
                    constraints,
                    null,
                    new GoalEndState(0.0, robotHeading),
                    false);

            path.preventFlipping = true;
            return path;
        }
    }
}
