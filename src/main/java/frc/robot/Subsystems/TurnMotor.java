package frc.robot.Subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.DriveConstants.TurnMotorConstants;

public class TurnMotor {

    private SparkFlex m_motor;
    private AbsoluteEncoder m_encoder;
    private SparkClosedLoopController m_closedLoopController;

    public TurnMotor (int motorID, boolean inverted) {
        m_motor = new SparkFlex(motorID, MotorType.kBrushless);
        m_encoder = m_motor.getAbsoluteEncoder();
        m_closedLoopController = m_motor.getClosedLoopController();

        SparkMaxConfig motorConfig = new SparkMaxConfig();

        motorConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(TurnMotorConstants.kCurrentLimit);
        motorConfig.absoluteEncoder
            .inverted(inverted)
            .positionConversionFactor(TurnMotorConstants.kPositionConversionFactor.in(Units.Radians))
            .velocityConversionFactor(TurnMotorConstants.kVelocityConversionFactor.in(Units.RadiansPerSecond));
        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
            .pid(
                TurnMotorConstants.PID.kP, 
                TurnMotorConstants.PID.kI,
                TurnMotorConstants.PID.kD
            )
            .outputRange(-1, 1)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, TurnMotorConstants.kPositionConversionFactor.in(Units.Radians));

        m_motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromRadians(m_encoder.getPosition());
    }

    public void setAngle(Rotation2d targetAngle){
        m_closedLoopController.setSetpoint(targetAngle.getRadians(), ControlType.kPosition);
    }
}
