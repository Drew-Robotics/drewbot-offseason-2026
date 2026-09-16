package frc.robot.subsystems.drive;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.DriveConstants.DriveMotorConstants;

//MOTOR NOTES: drive motors have no external encoders, everything's brushless, everything's flex controllers

public class DriveMotor {
    private final SparkFlex m_motor;
    private final RelativeEncoder m_encoder;
    private final SparkClosedLoopController m_closedLoopController;

    public DriveMotor(int motorID, boolean inverted) {
        m_motor = new SparkFlex(motorID, MotorType.kBrushless);
        m_encoder = m_motor.getEncoder();
        m_closedLoopController = m_motor.getClosedLoopController();

        SparkFlexConfig motorConfig = new SparkFlexConfig();

        motorConfig
            .idleMode(DriveMotorConstants.kIdleMode)
            .smartCurrentLimit(DriveMotorConstants.kCurrentLimit)
            .inverted(inverted);
        motorConfig.encoder
            .positionConversionFactor(DriveMotorConstants.kPositionConversionFactor)
            .velocityConversionFactor(DriveMotorConstants.kVelocityConversionFactor)
            //default velocity filtering adds a lot of lag which messes up sysid + the velocity loop
            .quadratureMeasurementPeriod(10)
            .quadratureAverageDepth(2)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);
        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(DriveMotorConstants.PID.kP, DriveMotorConstants.PID.kI, DriveMotorConstants.PID.kD)
            .outputRange(-1, 1);
        motorConfig.closedLoop.feedForward
            .kS(DriveMotorConstants.Feedforward.kS)
            .kV(DriveMotorConstants.Feedforward.kV);
        motorConfig.signals //10ms on the stuff URCL needs for sysid
            .primaryEncoderPositionPeriodMs(10)
            .primaryEncoderVelocityPeriodMs(10)
            .appliedOutputPeriodMs(10)
            .busVoltagePeriodMs(10)
            .outputCurrentPeriodMs(10);

        m_motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_encoder.setPosition(0);
    }

    public LinearVelocity getVelocity() {
        return Units.MetersPerSecond.of(m_encoder.getVelocity());
    }

    public Distance getPosition() {
        return Units.Meters.of(m_encoder.getPosition());
    }

    public void setVelocity(LinearVelocity commandedVelocity) {
        m_closedLoopController.setSetpoint(commandedVelocity.in(Units.MetersPerSecond), ControlType.kVelocity);
    }

    public void setVoltage(Voltage voltage) {
        m_motor.setVoltage(voltage);
    }

    /** Stator (phase) current, same thing smartCurrentLimit limits. */
    public Current getCurrent() {
        return Units.Amps.of(m_motor.getOutputCurrent());
    }

    /** Changes only the smart current limit, doesn't touch anything else or burn flash. For characterization. */
    public void setCurrentLimit(int amps) {
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(amps);
        m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /** What the flex actually has for its current limits, to check a requested limit took. */
    public String getCurrentLimitsString() {
        return "smart=" + m_motor.configAccessor.getSmartCurrentLimit() + "A secondary=" + m_motor.configAccessor.getSecondaryCurrentLimit() + "A";
    }

    public Voltage getAppliedVoltage() {
        return Units.Volts.of(m_motor.getAppliedOutput() * m_motor.getBusVoltage());
    }
}
