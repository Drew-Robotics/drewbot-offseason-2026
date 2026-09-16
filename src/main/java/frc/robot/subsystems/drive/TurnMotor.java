package frc.robot.subsystems.drive;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.DriveConstants.TurnMotorConstants;

/**
 * Closed loop runs in the flex on the vortex's internal encoder (less noise + lag than the analog).
 * The internal encoder only knows relative position, so we seed it from the thrifty absolute encoder
 * at boot, whenever the robot is disabled and the module is still, and if the flex ever resets.
 */
public class TurnMotor {
    private final SparkFlex m_motor;
    private final SparkClosedLoopController m_closedLoopController;
    private final SparkAnalogSensor m_absoluteEncoder;
    private final RelativeEncoder m_motorEncoder;

    //offset only matters when seeding. after that the internal encoder reads robot-relative angle directly
    private final Rotation2d m_offset;

    public TurnMotor(int motorID, boolean inverted, Rotation2d offset) {
        m_motor = new SparkFlex(motorID, MotorType.kBrushless);
        m_closedLoopController = m_motor.getClosedLoopController();
        m_absoluteEncoder = m_motor.getAnalog();
        m_motorEncoder = m_motor.getEncoder();
        m_offset = offset;

        SparkFlexConfig motorConfig = new SparkFlexConfig();

        motorConfig
            .idleMode(TurnMotorConstants.kIdleMode)
            .smartCurrentLimit(TurnMotorConstants.kCurrentLimit)
            .inverted(inverted);
        motorConfig.analogSensor
            .inverted(TurnMotorConstants.kEncoderInverted)
            .positionConversionFactor(TurnMotorConstants.kAnalogPositionConversionFactor)
            .velocityConversionFactor(TurnMotorConstants.kAnalogVelocityConversionFactor);
        motorConfig.encoder
            .positionConversionFactor(TurnMotorConstants.kMotorPositionConversionFactor)
            .velocityConversionFactor(TurnMotorConstants.kMotorVelocityConversionFactor)
            .quadratureMeasurementPeriod(10)
            .quadratureAverageDepth(2)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);
        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(
                TurnMotorConstants.PID.kP,
                TurnMotorConstants.PID.kI,
                TurnMotorConstants.PID.kD
            )
            .outputRange(-1, 1)
            //internal encoder never wraps on its own, the flex takes the short way around within this range
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(-Math.PI, Math.PI);
        motorConfig.signals //10ms on the stuff URCL needs for sysid
            .primaryEncoderPositionPeriodMs(10)
            .primaryEncoderVelocityPeriodMs(10)
            .appliedOutputPeriodMs(10)
            .busVoltagePeriodMs(10)
            .analogPositionPeriodMs(20)
            .analogVoltagePeriodMs(20);

        m_motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_motor.clearFaults();
        seedFromAbsolute();
    }

    /** Module angle relative to the robot, -180 to 180. */
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(MathUtil.angleModulus(m_motorEncoder.getPosition()));
    }

    /** Takes an angle relative to the robot. */
    public void setAngle(Rotation2d targetAngle) {
        m_closedLoopController.setSetpoint(targetAngle.getRadians(), ControlType.kPosition);
    }

    /** Absolute encoder angle relative to the robot (offset applied). */
    public Rotation2d getAbsoluteAngle() {
        return getRawAbsoluteAngle().minus(m_offset);
    }

    /** Sensor angle with no offset applied, this is what you copy into the offset constants. */
    public Rotation2d getRawAbsoluteAngle() {
        return Rotation2d.fromRadians(m_absoluteEncoder.getPosition());
    }

    public void seedFromAbsolute() {
        m_motorEncoder.setPosition(getAbsoluteAngle().getRadians());
    }

    /** True if the flex rebooted (brownout etc) and lost its seeded position. Clears the flag. */
    public boolean checkAndClearReset() {
        if (m_motor.getStickyWarnings().hasReset) {
            m_motor.clearFaults();
            return true;
        }
        return false;
    }

    public void setVoltage(Voltage voltage) {
        m_motor.setVoltage(voltage);
    }

    public Voltage getAppliedVoltage() {
        return Units.Volts.of(m_motor.getAppliedOutput() * m_motor.getBusVoltage());
    }

    public AngularVelocity getVelocity() {
        return Units.RadiansPerSecond.of(m_motorEncoder.getVelocity());
    }

    public double getEncoderVoltage() {
        return m_absoluteEncoder.getVoltage();
    }
}
