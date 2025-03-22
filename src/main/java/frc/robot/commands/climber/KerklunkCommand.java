package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KerklunkSubsystem;


public class KerklunkCommand extends Command{
    //Declares variables
    private final KerklunkSubsystem kerklunk;
    private final double targetAngle;

    public KerklunkCommand(KerklunkSubsystem subsystem, double targetAngle) {
        //Imports values
        this.kerklunk = subsystem; 
        this.targetAngle = targetAngle;
        // Declare subsystem dependency so no other command can use it at the same time.
        addRequirements(kerklunk);

    }
    @Override
    public void initialize(){
        //Prepares to set the kerklunk's angle to a specific angle
        kerklunk.setAngle(targetAngle);
    }

    @Override
    public void execute(){
        //Sets the kerklunk's angle to a specific angle
        kerklunk.setAngle(targetAngle);
    }

    @Override 
    //If something gets in the way of the kerklunk, ends the command as an emergency
    public void end(boolean interrupted) {
        //kerklunk.zeroAngle(); 
        // enable if servo needs to go to zero when ending; default probably is no
    }

    @Override
    public boolean isFinished()
    {
        return true; //Checks if it is at its target position and stops the system
    }
    
}
