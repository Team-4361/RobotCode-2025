package frc.robot.commands;


import frc.robot.subsystems.algaesubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeSuckCommand extends Command {
    private final algaesubsystem algae;

    public AlgaeSuckCommand(algaesubsystem subsystem) {
        this.algae = subsystem;
        addRequirements(algae);
    }

    @Override
    public void initialize() {
        algae.suck();
    }

    @Override
    public void execute() {
        algae.suck();
    }

    @Override
    public void end(boolean interrupted) {
        algae.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
