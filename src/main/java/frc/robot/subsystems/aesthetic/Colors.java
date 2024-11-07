package frc.robot.subsystems.aesthetic;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import frc.robot.Constants.Lights;


public class Colors {
	public enum Effect {
		LARSON,
		FLOW,
		CHROMA,
		BREATHE,
		REN_SPECIAL
	}

	Lights light = new Lights();
	CANdle rgb = new CANdle(light.id);

	public Colors() {
		CANdleConfiguration config = new CANdleConfiguration();
		config.stripType = LEDStripType.RGB;
		config.brightnessScalar = light.brightnessScalar;

		rgb.configAllSettings(config); //Applies all settings for
	}	

	public void startRGB(Effect effects) {
		Animation animation = switch (effects) {
			case LARSON -> new LarsonAnimation(255, 255, 255);
			case FLOW -> new ColorFlowAnimation(255, 255, 255);
			case CHROMA -> new RainbowAnimation();
			case BREATHE -> new SingleFadeAnimation(235, 209, 39);
			case REN_SPECIAL -> new TwinkleAnimation(235, 209, 39);
		};

		rgb.animate(animation);
	}	
}
