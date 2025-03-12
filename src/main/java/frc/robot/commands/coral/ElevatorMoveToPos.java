package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorMoveToPos extends Command {
    
    private final ElevatorSubsystem elevator;
    private final double targetHeight;

    public ElevatorMoveToPos(ElevatorSubsystem elevator, double height) {
        this.elevator = elevator;
        this.targetHeight = height;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setTarget(targetHeight);
    }

    @Override
    public void execute() {
        elevator.setTarget(targetHeight);
    }

    @Override
    public boolean isFinished() {
        return elevator.atTarget(); // Check if it's at the target position
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            elevator.stopElevator();
        }
    }
}
