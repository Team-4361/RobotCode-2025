package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.Constants.climberConstants;

public class WinchSubsystem extends SubsystemBase {
    private SparkMax winchMotor;
    private RelativeEncoder winchEncoder;
   // private Joystick driverStationJoystick;
   // private DigitalInput limitSwitch;
    private PIDController winchPID;


    private static final int LIMIT_SWITCH_PORT = 0;

    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    
    private double targetPosition = 0.0;
    

    public WinchSubsystem() {
        winchMotor = new SparkMax(Constants.climberConstants.WINCH_MOTOR_ID, MotorType.kBrushless);
        winchEncoder = winchMotor.getEncoder();
        //driverStationJoystick = new Joystick(0);
        //limitSwitch = new DigitalInput(LIMIT_SWITCH_PORT);
        winchPID = new PIDController(kP, kI, kD);
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
        targetPosition = 0.0;
    }


    public void winchMoveUp() {
        winchMotor.set(Constants.climberConstants.WINCH_SPEED);
        targetPosition = 30.0;
    }


    public void stopWinch() {
        winchMotor.set(0);
    }
}