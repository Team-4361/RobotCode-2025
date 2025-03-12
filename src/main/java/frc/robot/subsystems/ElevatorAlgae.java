/*package frc.robot.subsystems;
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

public class ElevatorAlgae extends SubsystemBase {
    private final SparkMax rotateMax;
    private final SparkMax outerSparkMax;
    private final RelativeEncoder encoder;
    private final PIDController pidController;

    private double targetPosition = 0.0;

    public ElevatorAlgae() {
        // Initialize motors
        rotateMax = new SparkMax(Constants.AE.ROTATE_MOTOR_ID, MotorType.kBrushless);
        outerSparkMax = new SparkMax(Constants.AE.OUTER_MOTOR_ID, MotorType.kBrushless);

        // Configure motors
        SparkMaxConfig config = new SparkMaxConfig();
        config.encoder.positionConversionFactor(0.0143);
        config.idleMode(IdleMode.kBrake);

        rotateMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        outerSparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // Initialize encoder
        encoder = rotateMax.getEncoder();

        // Initialize PID Controller
        pidController = new PIDController(Constants.AE.kP, Constants.AE.kI, Constants.AE.kD);
        pidController.setTolerance(Constants.AE.POSITION_TOLERANCE);

        if (Constants.isDebug) {
            // Send initial PID values to SmartDashboard
            SmartDashboard.putNumber("PID/kP", Constants.AE.kP);
            SmartDashboard.putNumber("PID/kI", Constants.AE.kI);
            SmartDashboard.putNumber("PID/kD", Constants.AE.kD);
            SmartDashboard.putNumber("Algae/Target Position", targetPosition);
        }
    }

    public void setMotor(double speed)
    {
        rotateMax.set(speed);
    }
    /** Sets the target position for PID control */
   /*  public void setTargetPosition(double position) {
        targetPosition = position;
        pidController.reset();
    }

    @Override
    public void periodic() {
        double currentPosition = encoder.getPosition();
        double pidOutput = pidController.calculate(currentPosition, targetPosition);

        // Limit motor power
        pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));

        // Only move if outside tolerance
        if (!pidController.atSetpoint()) {
            rotateMax.set(pidOutput);
        } else {
            rotateMax.set(0);
        }

        if (Constants.isDebug) {
            // Update PID values from SmartDashboard
            pidController.setP(SmartDashboard.getNumber("PID/kP", Constants.AE.kP));
            pidController.setI(SmartDashboard.getNumber("PID/kI", Constants.AE.kI));
            pidController.setD(SmartDashboard.getNumber("PID/kD", Constants.AE.kD));

            // Display real-time data
            SmartDashboard.putNumber("Algae/Current Position", currentPosition);
            SmartDashboard.putNumber("Algae/Target Position", targetPosition);
            SmartDashboard.putNumber("Algae/PID Output", pidOutput);
        }
    }

    /** Moves algae out (extrudes) */
   /* * /public void extrude() {
        setMotors(-Constants.AE.AE_SPEED);
    } 

    /** Pulls algae in (sucks) **/
    /*public void suck() {
        setMotors(Constants.AE.AE_SPEED);
    }

    /** Stops all motors **/
   /*  public void stop() {
        setMotors(0);
        outerSparkMax.stopMotor();

    }

     /** Stops vertical movement **/
    /*public void stopUpAndDown() {
     rotateMax.stopMotor();
    } 

    /** Helper method to set left and right motors safely **/
    /*private void setMotors(double speed) {
        outerSparkMax.set(speed);
    }
    
}*/
