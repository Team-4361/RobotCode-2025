package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class algaesubsystem extends SubsystemBase {
    private final SparkMax sparkMax;
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private final RelativeEncoder encoder;
    private final PIDController pidController;
    private static final double GEAR_RATIO = 1.0; // change this
    private static final double COUNTS_PER_REV = 42.0; // 42 counts per revolution
    private static final double DEGREES_PER_MOTOR_REV = 360.0; // 1 motor rev = 360 degrees

    private static final double POSITION_CONVERSION_FACTOR = 1;
    private double targetPosition = 0.0;

    public algaesubsystem() {
        // Initialize motors
        leftMotor = new SparkMax(Constants.Algae.LEFT_MOTOR_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.Algae.RIGHT_MOTOR_ID, MotorType.kBrushless);
        sparkMax = new SparkMax(Constants.Algae.ALGAE_MOTOR_ID, MotorType.kBrushless);

        // Configure motors
        SparkMaxConfig config = new SparkMaxConfig();
        config.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR);
        config.idleMode(IdleMode.kBrake);

        sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialize encoder
        encoder = sparkMax.getEncoder();

        // Initialize PID Controller
        pidController = new PIDController(Constants.Algae.kP, Constants.Algae.kI, Constants.Algae.kD);
        pidController.setTolerance(Constants.Algae.POSITION_TOLERANCE);
        SmartDashboard.putNumber("algae position", encoder.getPosition());

        if (Constants.isDebug) {
            // Send initial PID values to SmartDashboard
            SmartDashboard.putNumber("PID/kP", Constants.Algae.kP);
            SmartDashboard.putNumber("PID/kI", Constants.Algae.kI);
            SmartDashboard.putNumber("PID/kD", Constants.Algae.kD);
            SmartDashboard.putNumber("Algae/Target Position", targetPosition);
        }
    }

    public void setMotor(double speed)
    {
        /* 
        if (encoder.getPosition() < 0.05  || encoder.getPosition() > 138.0) 
        {
            sparkMax.stopMotor();
        }
        else
        {
            sparkMax.set(speed);
        }
        
        
    */

        sparkMax.set(speed);
        
    }
    /** Sets the target position for PID control */
   /*  public void setTargetPosition(double position) {
        targetPosition = position;
        pidController.reset();
    }*/

    @Override
    public void periodic() {
        //double currentPosition = encoder.getPosition();
        //double pidOutput = pidController.calculate(currentPosition, targetPosition);
        //SmartDashboard.putNumber("algae position", encoder.getPosition());


        // Limit motor power
      // pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));

        // Only move if outside tolerance
       /*  if (!pidController.atSetpoint()) {
            sparkMax.set(pidOutput);
        } else {
            sparkMax.set(0);
        }*/

        if (Constants.isDebug) {
            // Update PID values from SmartDashboard
            //pidController.setP(SmartDashboard.getNumber("PID/kP", Constants.Algae.kP));
            //pidController.setI(SmartDashboard.getNumber("PID/kI", Constants.Algae.kI));
            //pidController.setD(SmartDashboard.getNumber("PID/kD", Constants.Algae.kD));

            // Display real-time data
            ///SmartDashboard.putNumber("Algae/Current Position", currentPosition);
            //SmartDashboard.putNumber("Algae/Target Position", targetPosition);
            //SmartDashboard.putNumber("Algae/PID Output", pidOutput);
        }
    }

    /** Moves algae out (extrudes) */
    public void extrude() {
        setMotors(-Constants.Algae.ALGAE_SPEED);
    }

    /** Pulls algae in (sucks) */
    public void suck() {
        setMotors(Constants.Algae.ALGAE_SPEED);
    }

    /** Stops all motors */
    public void stop() {
        setMotors(0);
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    /** Stops vertical movement */
    public void stopUpAndDown() {
        sparkMax.stopMotor();
    }

    /** Helper method to set left and right motors safely */
    private void setMotors(double speed) {
        leftMotor.set(-speed);
        rightMotor.set(speed);
    }
}
