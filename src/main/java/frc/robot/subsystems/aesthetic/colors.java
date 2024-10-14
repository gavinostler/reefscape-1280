package frc.robot.subsystems.aesthetic;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import frc.robot.Constants.Lights;


public class colors {
	Lights light = new Lights();
	CANdle rgb = new CANdle(light.id);

	public colors(){
		CANdleConfiguration config = new CANdleConfiguration();
		config.stripType = LEDStripType.RGB;
		config.brightnessScalar = light.brightnessScalar;

		rgb.configAllSettings(config); //Applies all settings for

		
	}	
	public void startRGB(String effects){
		switch(effects){
			case "LARSON": LarsonAnimation larson = new LarsonAnimation(255, 255, 255);rgb.animate(larson); break;
			case "FLOW": ColorFlowAnimation flow = new ColorFlowAnimation(255,255,255);rgb.animate(flow); break;
			case "CHROMA": RainbowAnimation chroma = new RainbowAnimation();rgb.animate(chroma); break;
			case "BREATHE": SingleFadeAnimation breathe = new SingleFadeAnimation(235, 209, 39);rgb.animate(breathe); break;
			case "REN_SPECIAL": TwinkleAnimation ren = new TwinkleAnimation(235, 209, 39); rgb.animate(ren); break;
			default: rgb.setLEDs(235, 209, 39);	
		}
	}	
}
