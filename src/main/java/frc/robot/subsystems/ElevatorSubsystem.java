package frc.robot.subsystems;

import java.lang.annotation.Target;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
//import com.fasterxml.jackson.databind.cfg.ContextAttributes;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private SparkMax leftMotor;
    private SparkMax rightMotor;
    // private static final double POSITION_TOLERANCE = 0.02;
    private PIDController pidController1;

    public final RelativeEncoder lEncoder;
    public final RelativeEncoder rEncoder;
    //private final ElevatorFeedforward m_feedForward;
    // stuff from old talon code that idk is needed
    // private double integral = 0.0;
    // private double previousError = 0.0;
    private double targetPosition = 0.0; // Target position for PID control
    //private double pidOutput1 = 0.0;

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

    public void setTarget(double position) {
        targetPosition = position;
    }

    public double getTarget() {
        return targetPosition;
    }

    public boolean atTarget() {
        return pidController1.atSetpoint();


    }
    // public boolean aroundHeight(double height) {
    //     return aroundHeight(height, ElevatorConstants.kElevatorDefaultTolerance);
    // }

    // public boolean aroundHeight(double height, double tolerance) {
    //     return MathUtil.isNear(height, getPositionMeters(), tolerance);
    // }


    public void elevatorMoveUp() {
        // TODO: put this in constants
        if (lEncoder.getPosition() > 146.8 || rEncoder.getPosition() < -146.8) {
            leftMotor.stopMotor();
            rightMotor.stopMotor();
        } else {
            leftMotor.set(Constants.Coral.ELEVATOR_SPEED);
            rightMotor.set(-Constants.Coral.ELEVATOR_SPEED);

        }

    }

    public void elevatorMoveDown() {
        if (lEncoder.getPosition() < 4 || rEncoder.getPosition() > -4) {
            leftMotor.stopMotor();
            rightMotor.stopMotor();
        } else {
            leftMotor.set(-Constants.Coral.ELEVATOR_SPEED);
            rightMotor.set(Constants.Coral.ELEVATOR_SPEED);
        }

    }
    public void elevatorPosUp(double position, double speed)
    {
        leftMotor.set(speed);
        rightMotor.set(-speed);
        if(lEncoder.getPosition() > position && rEncoder.getPosition() < -position)
        {
            stopElevator();
        }
    }
    public void elevatorPosDown(double position, double speed)
    {
        leftMotor.set(-speed);
        rightMotor.set(speed);
        if(lEncoder.getPosition() < position && rEncoder.getPosition() > -position)
        {
            stopElevator();
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
        //SmartDashboard.putNumber("Left Encoder Position", lEncoder.getPosition());
        //SmartDashboard.putNumber("Right Encoder Position", rEncoder.getPosition());
        //SmartDashboard.putNumber("target pos", targetPosition);
        //SmartDashboard.putNumber("pid output", pidOutput1);

        //todo - Softlock so it stops motor if it is below a position or above a position.


        if (lEncoder.getPosition() > 146.8 || rEncoder.getPosition() < -146.8 || lEncoder.getPosition() < 4 || rEncoder.getPosition() > -4) {
            leftMotor.stopMotor();
            rightMotor.stopMotor();


        }
    }
}