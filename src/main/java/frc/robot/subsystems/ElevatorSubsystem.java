package frc.robot.subsystems;


import javax.swing.text.Position;

import org.opencv.core.Point;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import java.lang.annotation.Target;*/

import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    //Declares the SparkMaxes
    private SparkMax leftMotor;
    private SparkMax rightMotor;
    public final RelativeEncoder lEncoder;
    public final RelativeEncoder rEncoder;
    private double test;
    /**All pid and feedforward stuff that is not needed currently */
    //private final ElevatorFeedforward m_feedForward;
    // private double integral = 0.0;
    // private double previousError = 0.0;
    //private double targetPosition = 0.0; // Target position for PID control
    
    public ElevatorSubsystem() {
        leftMotor = new SparkMax(Constants.Coral.LEFT_ELEVATOR_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.Coral.RIGHT_ELEVATOR_ID, MotorType.kBrushless);
        lEncoder = leftMotor.getEncoder();
        rEncoder = rightMotor.getEncoder();
        SparkMaxConfig config = new SparkMaxConfig();

       /*  m_feedForward = new ElevatorFeedforward(ElevatorConstants.kElevatorkS,
                ElevatorConstants.kElevatorkG,
                ElevatorConstants.kElevatorkV,
                ElevatorConstants.kElevatorkA);*/

        config.idleMode(IdleMode.kBrake);
        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //pidController1 = new PIDController(Constants.ElevatorConstants.kElevatorKp,
                //Constants.ElevatorConstants.kElevatorKi, Constants.ElevatorConstants.kElevatorKd);
        //pidController1.setTolerance(0.5);
    }

/*   public void setTarget(double position) {
        targetPosition = position;
    }

    public double getTarget() {
        return targetPosition; //Tells the elevator to move to the position
    }

    public boolean atTarget() {
        return pidController1.atSetpoint(); //Tells the elevator its at the position


    }
        */
    // public boolean aroundHeight(double height) {
    //     return aroundHeight(height, ElevatorConstants.kElevatorDefaultTolerance);
    // }

    // public boolean aroundHeight(double height, double tolerance) {
    //     return MathUtil.isNear(height, getPositionMeters(), tolerance);
   // }


    public void elevatorMoveUp() {
        // TODO: put this in constants
        if (lEncoder.getPosition() > Constants.Coral.UP_LIMIT || rEncoder.getPosition() < -Constants.Coral.UP_LIMIT) {
            leftMotor.stopMotor();
            rightMotor.stopMotor();
        } else {
            leftMotor.set(Constants.Coral.ELEVATOR_SPEED);
            rightMotor.set(-Constants.Coral.ELEVATOR_SPEED);
        }

        

    }

    public void elevatorMoveDown() {

        if (lEncoder.getPosition() < 1.5 || rEncoder.getPosition() > 1.5) {
            leftMotor.stopMotor();
            rightMotor.stopMotor(); //Stops the motor when its at the position
        } else {
            leftMotor.set(-Constants.Coral.ELEVATOR_SPEED); //moves the motor until its at the position
            rightMotor.set(Constants.Coral.ELEVATOR_SPEED);
        }

    }
    public void elevatorPosUp(double position, double speed)
    {
        test = position;
        leftMotor.set(speed); //sets the elevator speed
        rightMotor.set(-speed);
        if(lEncoder.getPosition() > position && rEncoder.getPosition() < -position)
        {
            leftMotor.stopMotor();
            rightMotor.stopMotor();
        }
    }
    public void elevatorPosDown(double position, double speed)
    {
        
        leftMotor.set(-speed); //sets the elevator
        rightMotor.set(speed);
        if(lEncoder.getPosition() < position && rEncoder.getPosition() > -position)
        {
            stopElevator(); //when its at position, stops the elevator
        }

    }

    public void stopElevator() {
        leftMotor.stopMotor(); 
        rightMotor.stopMotor();
    }

    /** Runs the PID loop to move the motor to the target position */
    @Override
    public void periodic() 
    {
        SmartDashboard.putNumber("Left Encoder Position", lEncoder.getPosition());
        SmartDashboard.putNumber("Right Encoder Position", rEncoder.getPosition());
        SmartDashboard.putNumber("targetPos", test );
        SmartDashboard.putBoolean("reach target?", lEncoder.getPosition() >= test || rEncoder.getPosition() <= -test );
        //todo - Softlock so it stops motor if it is below a position or above a position.
        /*if (lEncoder.getPosition() > Constants.Coral.UP_LIMIT || rEncoder.getPosition() < Constants.Coral.UP_LIMIT || lEncoder.getPosition() < 4 || rEncoder.getPosition() > -4) {
            leftMotor.stopMotor();
            rightMotor.stopMotor();
        }*/
    }
}