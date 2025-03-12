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
        winch.winchMoveDown();
    }
    
    public void execute()
    {
        winch.winchMoveDown();
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