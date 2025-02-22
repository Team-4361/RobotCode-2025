package frc.robot.commands.algae;

import frc.robot.subsystems.algaesubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeDownCommand extends Command {
    private final algaesubsystem algae;

    public AlgaeDownCommand(algaesubsystem subsystem) {
        this.algae = subsystem;
        // Declare subsystem dependency so no other command can use it at the same time.
        addRequirements(algae);
    }

    // Called once when the command is initially scheduled.
    @Override
    public void initialize() {
        algae.setTargetPosition(-600);
    }

    // Called repeatedly while the command is scheduled.
    @Override
    public void execute() {
        // Optionally, call extrude continuously (if needed)
        algae.setTargetPosition(-600);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        algae.stopUpAndDown();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;  // It will run until the trigger is released.
    }
}
