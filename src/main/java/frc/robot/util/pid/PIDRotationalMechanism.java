package frc.robot.util.pid;


import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.math.GearRatio;
import frc.robot.util.motor.MotorModel;

public class PIDRotationalMechanism extends PIDMechanismBase {
    private final GearRatio ratio;
    private final RotationUnit unit;

    /**
     * Constructs a new {@link PIDRotationalMechanism}.
     *
     * @param motorId       The {@link CANSparkMax} motor ID to use.
     * @param constants     The {@link PIDConstants} to use.
     * @param model         The {@link MotorModel} of the {@link CANSparkMax} motor.
     * @param moduleName    The {@link String} module name
     * @param tuningEnabled If PID {@link SmartDashboard} tuning is enabled.
     * @param ratio         The {@link GearRatio} of the {@link PIDRotationalMechanism}.
     * @param unit          The conversion unit of the {@link PIDRotationalMechanism}. <b>MUST BE CONSISTENT!</b>
     */
    public PIDRotationalMechanism(int motorId,
                                  PIDConstants constants,
                                  MotorModel model,
                                  String moduleName,
                                  boolean tuningEnabled,
                                  GearRatio ratio,
                                  RotationUnit unit,
                                  boolean rpmControl) {
        super(motorId, constants, model, moduleName, tuningEnabled, rpmControl);
        this.ratio = ratio;
        this.unit = unit;
    }

    public enum RotationUnit { DEGREES, RADIANS, ROTATIONS }

    /**
     * @param motorRotations The motor rotations as reported by the {@link Encoder}.
     * @return The current value which should be used for current positional based control. It MUST be in the
     * <b>SAME UNIT</b> as the Target Position.
     */
    @Override
    protected double getCurrentPosition(double motorRotations) {
        double adjustedRotations = ratio.getFollowerRotations(motorRotations);

        return switch (unit) {
            case DEGREES -> Units.rotationsToDegrees(adjustedRotations);
            case RADIANS -> Units.rotationsToRadians(adjustedRotations);
            default -> adjustedRotations;
        };
    }
}
