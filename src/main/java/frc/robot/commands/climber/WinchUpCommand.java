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
        winch.winchMoveUp();
    }
    
    public void execute()
    {
        winch.winchMoveUp();
    }

    public void end() 
    {
        winch.stopWinch();
    }

    @Override
    public void end(boolean interrupted) {
        winch.stopWinch();
    }


    @Override
    public boolean isFinished()
    {
        return false; //something... idk yet 
    }
}