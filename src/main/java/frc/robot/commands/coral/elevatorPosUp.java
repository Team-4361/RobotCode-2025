package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class elevatorPosUp extends Command {
    
    private final ElevatorSubsystem elevator;
    private final double targetHeight;
    private final double eSpeed;

    public elevatorPosUp(ElevatorSubsystem elevator, double height, double speed) {
        this.elevator = elevator;
        this.targetHeight = height;
        eSpeed =speed;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.elevatorPosUp(targetHeight, eSpeed);
    }

    @Override
    public void execute() {
        elevator.elevatorPosUp(targetHeight, eSpeed);
    }

    @Override
    public boolean isFinished() {
        return elevator.lEncoder.getPosition() == targetHeight || elevator.rEncoder.getPosition() == -targetHeight; // Check if it's at the target position
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            elevator.stopElevator();
        }
    }
}
