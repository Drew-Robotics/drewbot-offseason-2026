package frc.robot.subsystems.drive;

import java.lang.module.Configuration;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.DriveConstants.DriveMotorConstants;

//MOTOR NOTES: drive motors have no encoders, everything's brushless, everything's flex controllers


public class DriveMotor {
    private SparkFlex m_motor;
    private RelativeEncoder m_encoder;
    private SparkClosedLoopController m_closedLoopController;

    public DriveMotor (int motorID, boolean inverted) {
        m_motor = new SparkFlex(motorID, MotorType.kBrushless);
        m_encoder = m_motor.getEncoder();
        m_closedLoopController = m_motor.getClosedLoopController();

        SparkFlexConfig motorConfig = new SparkFlexConfig();

        motorConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(60);
        motorConfig.encoder
            .positionConversionFactor(DriveConstants.DriveMotorConstants.kPositionConversionFactor.in(Units.Meters))
            .velocityConversionFactor(DriveConstants.DriveMotorConstants.kVelocityConversionFactor.in(Units.MetersPerSecond));
        motorConfig.closedLoop
            .pid(DriveMotorConstants.PID.kP, DriveMotorConstants.PID.kI, DriveMotorConstants.PID.kD)
            .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
            .outputRange(-1, 1);

        m_motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public LinearVelocity getVelocity () {
        return Units.MetersPerSecond.of(m_encoder.getVelocity());
    }
    public Distance getPosition () {
        return Units.Meters.of(m_encoder.getPosition());
    }

    public void setVelocity(LinearVelocity commandedVelocity){
        m_closedLoopController.setSetpoint(commandedVelocity.in(Units.MetersPerSecond), ControlType.kVelocity);
    }
}
