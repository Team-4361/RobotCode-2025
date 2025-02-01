package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class algaesubsystem extends SubsystemBase {
    private SparkMax leftMotor;
    private SparkMax rightMotor;

    public algaesubsystem() {

        leftMotor = new SparkMax(Constants.Algae.LEFT_MOTOR_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.Algae.RIGHT_MOTOR_ID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake);

        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);



    }    

    public void exextude() {
        leftMotor.set(-Constants.Algae.ALGAE_SPEED); 
        rightMotor.set(-Constants.Algae.ALGAE_SPEED);
    }


    public void suck() {
        leftMotor.set(Constants.Algae.ALGAE_SPEED);
        rightMotor.set(Constants.Algae.ALGAE_SPEED);
    }
    
    public void stop() {
        leftMotor.set(0);
        rightMotor.set(0);
    }

}
