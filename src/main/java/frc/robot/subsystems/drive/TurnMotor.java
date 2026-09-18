package frc.robot.subsystems.drive;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.DriveConstants.TurnMotorConstants;

import com.revrobotics.spark.config.SparkFlexConfig;

public class TurnMotor {
    private SparkFlex m_motor;
    private SparkClosedLoopController m_closedLoopController;
    private SparkAnalogSensor m_encoder;
    private String m_name;
    private Rotation2d m_offset;

    public TurnMotor (String name, int motorID, boolean inverted, Rotation2d offset) {
        m_motor = new SparkFlex(motorID, MotorType.kBrushless);
        m_closedLoopController = m_motor.getClosedLoopController();
        m_encoder = m_motor.getAnalog();
        m_name = name;
        m_offset = offset;
        
        SparkFlexConfig motorConfig = new SparkFlexConfig();

        motorConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(TurnMotorConstants.kCurrentLimit);
        motorConfig.analogSensor
            .positionConversionFactor(DriveConstants.TurnMotorConstants.kPositionConversionFactor.in(Units.Radians))
            .velocityConversionFactor(DriveConstants.TurnMotorConstants.kVelocityConversionFactor.in(Units.RadiansPerSecond));
        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAnalogSensor)
            .pid(
                TurnMotorConstants.PID.kP, 
                TurnMotorConstants.PID.kI,
                TurnMotorConstants.PID.kD
            )
            .outputRange(-1, 1)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(-Math.PI, Math.PI);

        m_motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromRadians(m_encoder.getPosition()).minus(m_offset);
    }

    public void setAngle(Rotation2d targetAngle){
        m_closedLoopController.setSetpoint(targetAngle.getRadians(), ControlType.kPosition);
    }

    public double encoderVoltageCheck() {
        return m_encoder.getVoltage();
    }

    public void periodic() {
        SmartDashboard.putNumber(m_name + " Encoder" , encoderVoltageCheck());
    }
}