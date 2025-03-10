package frc.robot.commands.algae;


import frc.robot.subsystems.algaesubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeSuckCommand extends Command {
    private final algaesubsystem algae;

    public AlgaeSuckCommand(algaesubsystem subsystem) {
        this.algae = subsystem;
        // Declare subsystem dependency so no other command can use it at the same time.        
        addRequirements(algae);
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
}

