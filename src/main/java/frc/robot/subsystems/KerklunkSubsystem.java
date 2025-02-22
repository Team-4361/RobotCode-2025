package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;




 public class KerklunkSubsystem extends SubsystemBase {//it is a HS-322HD Servo that we called Kerklunk
    private Servo kerklunk;




    public KerklunkSubsystem() {
        kerklunk = new Servo(Constants.kerklunkConstants.kerklunkPort);
    }

    public void setAngle(double targetAngle) {
        kerklunk.setAngle(targetAngle);
    }
    public double getAngle()
    {
        return kerklunk.getAngle();
    }
    public void zeroAngle() {
        kerklunk.setAngle(0.0);
      
    }
} 