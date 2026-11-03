package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MiscConstants;

public class LEDs extends SubsystemBase {
    private final CANdle m_candle;

    public LEDs() {
        m_candle = new CANdle(MiscConstants.kCANdleID);

        RGBWColor purple = new RGBWColor(160, 0, 255);

        LarsonAnimation larsonAnim = new LarsonAnimation(8, 75);


        CANdleConfiguration conf = new CANdleConfiguration();

        conf.LED.StripType = StripTypeValue.GRB;
        conf.LED.BrightnessScalar = 1;

        m_candle.getConfigurator().apply(conf);

        m_candle.setControl(larsonAnim.withColor(purple));
    }
}
