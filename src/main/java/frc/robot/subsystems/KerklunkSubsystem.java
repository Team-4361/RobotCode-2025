package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



 public class KerklunkSubsystem extends SubsystemBase {//it is a HS-322HD Servo that we called Kerklunk
    private Servo kerklunk;
    public KerklunkSubsystem() {
        kerklunk = new Servo(Constants.climberConstants.KERKLUNK_PORT);
    }

    public void setAngle(double targetAngle) {
        kerklunk.setAngle(targetAngle); //sets the angle to the target angle
    }
    public double getAngle()
    {
        return kerklunk.getAngle();//gets the angle the kerklunk is in it
    }
    public void zeroAngle() {
        kerklunk.setAngle(0.0); //sets the angle to 0 and reverts the kerklunk
    }
      
    public void periodic()
    {
        
        SmartDashboard.putNumber("climber ", getAngle());
    }
} 