/*package frc.robot.commands.algae;

import frc.robot.Constants;
import frc.robot.subsystems.ElevatorAlgae;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeElevatorDownCommand extends Command {
    private final ElevatorAlgae algae;

    public AlgaeElevatorDownCommand(ElevatorAlgae subsystem) {
        this.algae = subsystem;
        // Declare subsystem dependency so no other command can use it at the same time.
        addRequirements(this.algae);
    }

    // Called once when the command is initially scheduled.
    @Override
    public void initialize() {
        //algae.setTargetPosition(-600);
        algae.setMotor(-Constants.AE.AE_SPEED);
    }

    // Called repeatedly while the command is scheduled.
    @Override
    public void execute() {
        // Optionally, call extrude continuously (if needed)
        //algae.setTargetPosition(-600);
        algae.setMotor(-Constants.AE.AE_SPEED);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //algae.stopUpAndDown();
        algae.setMotor(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;  // It will run until the trigger is released.
    }
}*/
