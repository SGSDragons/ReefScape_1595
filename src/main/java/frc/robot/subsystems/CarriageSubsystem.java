// package frc.robot.subsystems;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;

// import edu.wpi.first.wpilibj.Preferences;
// import edu.wpi.first.wpilibj.Servo;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.CarriageConstants;
// import frc.robot.Constants.LiftConstants;

// public class CarriageSubsystem extends SubsystemBase{

//     SparkMax coralMotor;
//     RelativeEncoder coralEncoder;

//     Servo direction;

//     LiftSubsystem lift;

//     public final double intakeSpeed = getPreference("IntakeSpeed", CarriageConstants.intakeSpeed);

//     public CarriageSubsystem(){

//         coralMotor = new SparkMax(CarriageConstants.coralMotorCanId, MotorType.kBrushless);
//         coralEncoder = coralMotor.getEncoder();

//         direction = new Servo(CarriageConstants.directionChannel);
//         direction.set(CarriageConstants.middle);

//         lift = new LiftSubsystem();
//     }

//     public Command testSparkMax(double power){
//         return run(() -> {
//             coralMotor.set(power);
//         });
//     }

//     public void spinCoralIntake(){
//         double speed = lift.reversed ? intakeSpeed : -intakeSpeed;
//         coralMotor.set(speed);
//     }

//     public void stopCoralMotor(){
//         coralMotor.set(0);
//     }

//     public Command middle(){

//         return run(() -> { 
//             direction.set(CarriageConstants.middle);
//             stopCoralMotor(); 
//         });
//     }

//     public Command pointRight(){

//         return run(() -> {
//             double invert = lift.reversed ? -1 : 1;
//             direction.set(CarriageConstants.pointRight*invert);
//             if (Math.abs(direction.getPosition() - CarriageConstants.pointRight) < 0.1) {
//                 spinCoralIntake();
//             }
//         });
//     }

//     public Command pointLeft(){

//         return run(() -> {
//             double invert = lift.reversed ? -1 : 1;
//             direction.set(CarriageConstants.pointLeft*invert);
//             if (Math.abs(direction.getPosition() - CarriageConstants.pointLeft) < 0.1) {
//                 spinCoralIntake();
//             }
//         });
//     }


//     @Override
//     public void periodic(){;
//         telemetry();
//     }

//     public void telemetry(){
//         SmartDashboard.putNumber("Rotation Motor Position", coralEncoder.getVelocity());
//         SmartDashboard.putNumber("Direction Servo Position", direction.getPosition());
//     }

//     private static double getPreference(String key, double fallback) {
//         return Preferences.getDouble("Carriage/" + key, fallback);
//     }
// }
