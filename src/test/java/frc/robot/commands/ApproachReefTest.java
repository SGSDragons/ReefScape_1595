package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;
import frc.robot.commands.ApproachReef.Approach;

import static org.junit.jupiter.api.Assertions.*;

class ApproachReefTest {

    @Test
    void getApproachAngleIdle() {

        assertEquals(Approach.LEFT, getApproach(-10, -1, 0, 0));

        assertEquals(Approach.LOWER_LEFT, getApproach(-1, -10, 0, 0));
        assertEquals(Approach.LOWER_RIGHT, getApproach(1, -10, 0, 0));

        assertEquals(Approach.RIGHT, getApproach(10, -1, 0, 0));
        assertEquals(Approach.RIGHT, getApproach(10, 1, 0, 0));

        assertEquals(Approach.UPPER_LEFT, getApproach(-1, 10, 0, 0));
        assertEquals(Approach.UPPER_RIGHT, getApproach(1, 10, 0, 0));

        assertEquals(Approach.LEFT, getApproach(-10, 1, 0, 0));
    }


    private Approach getApproach(double xLoc, double yLoc, double xSpeed, double ySpeed) {
        Translation2d location = new Translation2d(xLoc, yLoc);
        Translation2d velocity = new Translation2d(xSpeed, ySpeed);
        Translation2d reefPosition = new Translation2d();

        return Approach.best(location, velocity, reefPosition);
    }

}