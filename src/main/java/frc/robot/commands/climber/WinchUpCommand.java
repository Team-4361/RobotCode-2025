package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WinchSubsystem;

public class WinchUpCommand extends Command {
    private final WinchSubsystem winch;

    public WinchUpCommand(WinchSubsystem subsystem) {
        this.winch = subsystem;
        
        addRequirements(winch);
    }

    @Override
    public void initialize()
    {
        //Prepares to move the winch up
        winch.winchMoveUp();
    }
    
    public void execute()
    {
        //Move the winch up
        winch.winchMoveUp();
    }

    public void end() 
    {
         //whenever its done, it will stop moving the winch
        winch.stopWinch();
    }

    @Override
    public void end(boolean interrupted) {
        //if interrupted, stops the winch as an emergency
        winch.stopWinch();
    }


    @Override
    public boolean isFinished()
    {
        return false; //Checks if it is at its target position
    }
}