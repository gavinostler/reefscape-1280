package frc.robot.aesthetic;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.Lights;

public class Colors {
  public enum Effect {
    LARSON,
    FLOW,
    CHROMA,
    BREATHE,
    REN_SPECIAL,
    EW,
  }

  private static final CANdle candle = new CANdle(Lights.id);

  static {
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = Lights.brightnessScalar;
    candle.configAllSettings(config);
  }

  public Animation getEffectAnimation(Effect effect) {
    return switch (effect) {
      case LARSON -> new LarsonAnimation(255, 255, 255);
      case FLOW -> new ColorFlowAnimation(255, 255, 255);
      case CHROMA -> new RainbowAnimation(0.2, 0.2, Lights.leds);
      case BREATHE -> new SingleFadeAnimation(235, 209, 39);
      case REN_SPECIAL -> new TwinkleAnimation(235, 209, 39);
      case EW -> new SingleFadeAnimation(198, 184, 54); // disgusting color
    };
  }

  public void animateCandle(Effect effect) {
    Animation animation = getEffectAnimation(effect);
    candle.animate(animation);
  }

  public void staticColor(int red, int green, int blue) {
    candle.setLEDs(red, green, blue);
  }

  public void staticColor(Color8Bit color) {
    staticColor(color.red, color.green, color.blue);
  }
}
