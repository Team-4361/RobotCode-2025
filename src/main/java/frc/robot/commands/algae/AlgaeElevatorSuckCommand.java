/*package frc.robot.commands.algae;


import frc.robot.subsystems.ElevatorAlgae;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeElevatorSuckCommand extends Command {
    private final ElevatorAlgae algae;

    public AlgaeElevatorSuckCommand(ElevatorAlgae subsystem) {
        this.algae = subsystem;
        // Declare subsystem dependency so no other command can use it at the same time.        
        addRequirements(this.algae);
    }
    // Called once when the command is initially scheduled.
    @Override
    public void initialize() {
        algae.suck();
    }

    // Called repeatedly while the command is scheduled.
    @Override
    public void execute() {
        // Optionally, call suck continuously (if needed)        
        algae.suck();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        algae.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;   // It will run until the trigger is released.
    }
}*/
