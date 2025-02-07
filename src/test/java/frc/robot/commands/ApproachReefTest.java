package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.ApproachFactory;
import frc.robot.ApproachFactory.Approach;
import frc.robot.Constants;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class ApproachReefTest {

    ApproachFactory factory = new ApproachFactory(Translation2d.kZero);
    ApproachFactory redFactory = new ApproachFactory(Constants.Reefscape.redReef);

    @Test
    void getApproachAngleIdle() {
        assertEquals(factory.forAngleDegrees(-180), getApproach(-10, -1, 0, 0));

        assertEquals(factory.forAngleDegrees(-120), getApproach(-1, -10, 0, 0));
        assertEquals(factory.forAngleDegrees(-60), getApproach(1, -10, 0, 0));

        assertEquals(factory.forAngleDegrees(0), getApproach(10, -1, 0, 0));
        assertEquals(factory.forAngleDegrees(0), getApproach(10, 1, 0, 0));

        assertEquals(factory.forAngleDegrees(60), getApproach(1, 10, 0, 0));
        assertEquals(factory.forAngleDegrees(120), getApproach(-1, 10, 0, 0));

        assertEquals(factory.forAngleDegrees(180), getApproach(-10, 1, 0, 0));
    }


    private Approach getApproach(double xLoc, double yLoc, double xSpeed, double ySpeed) {
        Translation2d location = new Translation2d(xLoc, yLoc);
        Translation2d velocity = new Translation2d(xSpeed, ySpeed);
        return factory.bestApproach(location, velocity);
    }


    @Test
    void generatePath() {

        Approach lowerLeft = redFactory.forAngleDegrees(-120);
        Translation2d location = new Translation2d(12.7, 1.3);
        Translation2d velocity = new Translation2d(0, 0);

        lowerLeft.generatePath(location, velocity);
    }
}