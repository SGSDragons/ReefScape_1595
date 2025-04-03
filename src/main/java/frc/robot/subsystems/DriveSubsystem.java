package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;   
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

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
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY, double scale) {
        return run(() -> {});
    }

    /**
     * Command to drive the robot relative to the robot itself.
     * 
     * @param translationX A supplier where positive values drive forward
     * @param translationY A supplier where positive values strafe left
     * @param angularRotationX A supplier where positive values turn right
     * @return A drive command
     */
    public Command driveRelative(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
        return run(() -> {});
    }

    /**
     * Command to drive the robot using translation values and heading pointed to a reef
     *
     * @param translationX Translation in the X direction. Cubed for smoother controls.
     * @param translationY Translation in the Y direction. Cubed for smoother controls.
     * @return A drive command.
     */
    public Command driveTargeting(DoubleSupplier translationX, DoubleSupplier translationY) {
        return run(() -> {});
    }
}