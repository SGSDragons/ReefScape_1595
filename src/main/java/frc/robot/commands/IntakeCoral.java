// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.subsystems.CarriageSubsystem;
// import frc.robot.subsystems.ClimbSubsystem;
// import frc.robot.subsystems.CoralIntakeSubsystem;

// public class IntakeCoral extends Command{

//     CoralIntakeSubsystem coralIntake;
//     CarriageSubsystem carriageIntake;

//     IntakeCoral(CoralIntakeSubsystem coralIntake, CarriageSubsystem carriageIntake){
//         this.coralIntake = coralIntake;
//         this.carriageIntake = carriageIntake;
//         addRequirements(coralIntake, carriageIntake);
//     }

//     // Called when the command is initially scheduled.
//     public void initialize(){

//     }

//     // Called every time the scheduler runs while the command is scheduled.
//     public void execute(){
//         coralIntake.Intake();
//         carriageIntake.spinCoralIntake();
//     }

//     // Called once the command ends or is interrupted.
//     @Override
//     public void end(boolean interrupted) {
        
//     }

//     @Override
//     public boolean isFinished() {
//       return false;
//     }
    
// }
