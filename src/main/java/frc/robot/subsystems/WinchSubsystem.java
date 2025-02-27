package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class WinchSubsystem extends SubsystemBase {
    private SparkMax winchMotor;
    private RelativeEncoder winchEncoder;
   // private Joystick driverStationJoystick;
   // private DigitalInput limitSwitch;

    private static final double WINCH_SPEED = 0.8;
    
    private double integral = 0.0;
    private double previousError = 0.0;
    private double targetPosition = 0.0;
    

    public WinchSubsystem() {
        winchMotor = new SparkMax(Constants.climberConstants.WINCH_MOTOR_ID, MotorType.kBrushless);
        winchEncoder = winchMotor.getEncoder();
        //driverStationJoystick = new Joystick(0);
        //limitSwitch = new DigitalInput(LIMIT_SWITCH_PORT);
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
        winchMotor.set(-WINCH_SPEED);
        targetPosition = 0.0;
    }


    public void winchMoveUp() {
        winchMotor.set(WINCH_SPEED);
        targetPosition = 30.0;
    }


    public void stopWinch() {
        winchMotor.set(0);
    }
}