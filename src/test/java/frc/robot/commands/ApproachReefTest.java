package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import org.junit.jupiter.api.Test;
import frc.robot.commands.DynamicReefApproach.Approach;

import static org.junit.jupiter.api.Assertions.*;

class ApproachReefTest {

    DynamicReefApproach factory = new DynamicReefApproach(null, Translation2d.kZero);
    DynamicReefApproach redFactory = new DynamicReefApproach(null, Constants.Reefscape.redReef);

    @Test
    void getApproachAngleIdle() {
        assertEquals(factory.approaches.get(0), getApproach(-10, -1, 0, 0));

        assertEquals(factory.approaches.get(1), getApproach(-1, -10, 0, 0));
        assertEquals(factory.approaches.get(2), getApproach(1, -10, 0, 0));

        assertEquals(factory.approaches.get(3), getApproach(10, -1, 0, 0));
        assertEquals(factory.approaches.get(3), getApproach(10, 1, 0, 0));

        assertEquals(factory.approaches.get(4), getApproach(1, 10, 0, 0));
        assertEquals(factory.approaches.get(5), getApproach(-1, 10, 0, 0));

        assertEquals(factory.approaches.get(0), getApproach(-10, 1, 0, 0));
    }


    private Approach getApproach(double xLoc, double yLoc, double xSpeed, double ySpeed) {
        Translation2d location = new Translation2d(xLoc, yLoc);
        Translation2d velocity = new Translation2d(xSpeed, ySpeed);
        return factory.bestApproach(location, velocity);
    }


    @Test
    void generatePath() {

        Approach lowerLeft = redFactory.approaches.get(1);
        Translation2d location = new Translation2d(12.7, 1.3);
        Translation2d velocity = new Translation2d(0, 0);

        lowerLeft.generatePath(location, velocity);
    }
}