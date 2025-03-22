package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WinchSubsystem;

public class WinchDownCommand extends Command {
    private final WinchSubsystem winch;

    public WinchDownCommand(WinchSubsystem subsystem) {
        this.winch = subsystem;
        
        addRequirements(winch);
    }

    @Override
    public void initialize()
    {
        //Prepares to move the winch down
        winch.winchMoveDown();
    }
    
    public void execute()
    {
        //Moves the winch down
        winch.winchMoveDown();
    }

    public void end() 
    {
        //whenever its done, it will stop moving the winch
        winch.stopWinch();
    }

    @Override
    public void end(boolean interrupted) {
        //if interupted, stops the winch as an emergency
        winch.stopWinch();
    }


    @Override
    public boolean isFinished()
    {
        return false; //Checks if it is at its target position
    }
}