package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;

public class WinchSubsystem extends SubsystemBase {
    private SparkMax winchMotor;
    private RelativeEncoder winchEncoder;
   // private Joystick driverStationJoystick;
   // private DigitalInput limitSwitch;

    

    public WinchSubsystem() {
        winchMotor = new SparkMax(Constants.climberConstants.WINCH_MOTOR_ID, MotorType.kBrushless);
        winchEncoder = winchMotor.getEncoder(); //sets the winch encoder to the one being used
        //driverStationJoystick = new Joystick(0);
        //limitSwitch = new DigitalInput(LIMIT_SWITCH_PORT);
        winchEncoder.setPosition(0);

        /*
        double currentPos = winchEncoder.getPosition();
        double pidOutput = winchPID.calculate(currentPos, targetPosition);
        pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));
        winchMotor.set(pidOutput);
        */
    }


    @Override
    public void periodic() {}


    public void winchMoveDown() {
        winchMotor.set(-Constants.climberConstants.WINCH_SPEED);
    }


    public void winchMoveUp() {
        winchMotor.set(Constants.climberConstants.WINCH_SPEED);
    }


    public void stopWinch() {
        winchMotor.set(0);
    }
}