package frc.robot.subsystem;

import com.ctre.phoenix.led.CANdle;
enum effectType{
	Larson,Chroma,Twinkle,Strobe,
}


public class colors {
	public colors(enum type){

	CANdle rgb = new CANdle(27);

	CANdleConfiguration config = new CANdleConfiguration();

	config.stripType = LEDStripType.RGB;
	config.brightnessSclar = 0.8;

	rgb.configAllSettings(config);
	candle.setLEDs(255, 255, 255);

	case(type)

	}
	@override
	public larson()
}
