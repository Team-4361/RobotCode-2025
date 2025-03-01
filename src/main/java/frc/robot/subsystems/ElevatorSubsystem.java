// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
//import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
//import com.revrobotics.spark.SparkBase.ControlType;
//import com.revrobotics.spark.SparkBase.PersistMode;
//import com.revrobotics.spark.SparkBase.ResetMode;
//import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
//import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
//import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase
{

    private final DCMotor m_elevatorGearbox = DCMotor.getNEO(35);
    private final SparkMax l_motor = new SparkMax(Constants.Coral.LEFT_ELEVATOR_ID, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax r_motor = new SparkMax(Constants.Coral.RIGHT_ELEVATOR_ID, SparkLowLevel.MotorType.kBrushless);
    private final SparkMaxSim l_motorSim = new SparkMaxSim(l_motor, m_elevatorGearbox);
    private final SparkMaxSim r_motorSim = new SparkMaxSim(r_motor, m_elevatorGearbox);
    private final RelativeEncoder l_encoder = l_motor.getEncoder();
    private final RelativeEncoder r_encoder = r_motor.getEncoder();

    private final ProfiledPIDController m_controller = new ProfiledPIDController(Constants.ElevatorConstants.kElevatorKp,
            Constants.ElevatorConstants.kElevatorKi, 
            Constants.ElevatorConstants.kElevatorKd,
            new Constraints(Constants.ElevatorConstants.kElevatorMaxVelocity,
                    Constants.ElevatorConstants.kElevatorMaxAcceleration));
    private final ElevatorFeedforward m_feedForward = new ElevatorFeedforward(ElevatorConstants.kElevatorkS,
            ElevatorConstants.kElevatorkG,
            ElevatorConstants.kElevatorkV,
            ElevatorConstants.kElevatorkA);
    private ElevatorSim m_elevatorSim = null;
    // Sensors
/* 
    private final DigitalInput m_limitSwitchLow = new DigitalInput(9);
    private DIOSim m_limitSwitchLowSim = null;
*/
    public ElevatorSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(40)
                .openLoopRampRate(Constants.ElevatorConstants.kElevatorRampRate);

        l_motor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        //

        if (RobotBase.isSimulation()) {
            m_elevatorSim = new ElevatorSim(m_elevatorGearbox,
                    ElevatorConstants.kElevatorGearing,
                    Constants.ElevatorConstants.kCarriageMass,
                    ElevatorConstants.kElevatorDrumRadius,
                    Constants.ElevatorConstants.kMinElevatorHeightMeters,
                    Constants.ElevatorConstants.kMaxElevatorHeightMeters,
                    true,
                    0.0,
                    0.02,
                    0.0);

        }



    }

    /*public void simulationPeriodic() {
        //set input(voltage)
        m_elevatorSim.setInput(m_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        //update-every 20 milliseconds
        m_elevatorSim.update(0.02);

        m_motorSim.iterate(Elevator.convertDistanceToRotations(Meters.of(m_elevatorSim.getVelocityMetersPerSecond()))
                        .per(Second).in(RPM),
                RoboRioSim.getVInVoltage(),
                0.020);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));

        // Update lasercan sim.
        m_elevatorLaserCanSim.setMeasurementFullSim(new Measurement(
                LASERCAN_STATUS_VALID_MEASUREMENT,
                (int) (Math.floor(Meters.of(m_elevatorSim.getPositionMeters()).in(Millimeters)) +
                        ElevatorConstants.kLaserCANOffset.in(Millimeters)),
                0,
                true,
                m_laserCanTimingBudget.asMilliseconds(),
                m_laserCanROI
        ));

        //comment out for arm demo
        //Constants.elevatorMech.setLength(getPositionMeters());
        //Constants.elevatorCarriage.setPosition(AlgaeArmConstants.kAlgaeArmLength, getPositionMeters());
    }*/

    public double getPositionMeters() {
        return l_encoder.getPosition() * (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius)
                / ElevatorConstants.kElevatorGearing;
    }
    /*public double getPositionMetersR(){
      return r_encoder.getPosition() * (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius)
      / ElevatorConstants.kElevatorGearing;
    }*/

    public double getVelocityMetersPerSecond() {
        return (l_encoder.getVelocity() / 60) * (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius)
                / ElevatorConstants.kElevatorGearing;
    }
   /*  public double getVelocityMetersPerSecondR() {
      return (r_encoder.getVelocity() / 60) * (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius)
              / ElevatorConstants.kElevatorGearing;
  }*/

    public void reachGoal(double goal){
        double voltsOutput = MathUtil.clamp(
                m_feedForward.calculateWithVelocities(getVelocityMetersPerSecond(), m_controller.getSetpoint().velocity)
                + m_controller.calculate(getPositionMeters(), goal),
                -7,
                7);
        l_motor.setVoltage(voltsOutput);
        r_motor.setVoltage(voltsOutput);
        
    }

    public Command setGoal(double goal){
        return run(() -> reachGoal(goal));
    }

    public Command setElevatorHeight(double height){
        return setGoal(height).until(()->aroundHeight(height));
    }

    public boolean aroundHeight(double height){
        return aroundHeight(height, Constants.ElevatorConstants.ELEVATOR_TOLERANCE);
    }
    public boolean aroundHeight(double height, double tolerance){
        return MathUtil.isNear(height,getPositionMeters(),tolerance);
    }

     /**
     * Stop the control loop and motor output.
     */
    public void stop()
    {
        l_motor.set(0.0);
        r_motor.set(0.0);
    }
  public void updateTelemetry()
  {
    // Update elevator visualization with position
    //m_elevatorMech2d.setLength(RobotBase.isSimulation() ? m_elevatorSim.getPositionMeters() : m_encoder.getPosition());
  }

  @Override
  public void periodic()
  {
    updateTelemetry();
    
  }
}
