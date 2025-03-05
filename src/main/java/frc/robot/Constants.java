// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Pigeon2Configuration;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static String kCANbusName = "rio";
  public static final boolean kEnableSysId = true;

  // public static String kCANivore = "RENivore";
  public static class Operator {
    public static final int kDriverControllerPort = 0;
  }

  public static class Pigeon2 {
    static final int id = 26; // TODO: set can id
    static final Pigeon2Configuration config = null; // idk what it do
  }

  public static class Lights {
    public final int id = 27; // CAN ID for RGB
    public final double brightnessScalar = 1.0; // Brightness for Color
    public final int leds = 150;
  }

  public static class Climber {
    public static final int id = -1; // TODO: set talon id
    public static final int encoderId = -1; // TODO
    public static double maxValue = 1.0;
    public static double kRotateSpeed = 0.2;
  }
}
