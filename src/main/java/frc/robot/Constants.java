// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import frc.robot.subsystems.SwerveDriveSubsystem;


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

  public static class Operator {
    public static final int kDriverControllerPort = 0;
  }

  public static class Pigeon2 {
    static final int id = 26; // TODO: set can id
    static final Pigeon2Configuration config = null; // idk what it do
  }

  public static class Music {
    //ditto: Configurating the "Instruments"
    public final int mID1 = 1;
    public final int mID2 = 2;
    public final int mID3 = 3; 
    public final int mID4 = 4;
    public final int mID5 = 5;
    public final int mID6 = 6;
    public final int mID7 = 7;
    public final int mID8 = 8;
  }

  public static class Lights {
    public final int id = 27; // CAN ID for RGB
    public final double brightnessScalar = 0.8; //Brightness for Color
  }

  // This should really be auto generated after the motors are connected but, alas, i did not read the documentation.
  public static class Drivetrain {
    static double arvind = Math.PI / 2;

    public record Module (
      int driveID, int steerID, int encoderID,
      double encoderOffset, // rotations
      double xPos, double yPos // inches
    ) {}

    // TODO: set everything
    static final Module fl = new Module(1, 5, 63, arvind, 0, 1);
    static final Module fr = new Module(2, 6, 62, arvind, 1, 2);
    static final Module bl = new Module(3, 7, 61, arvind, 2, 3); // yaoi
    static final Module br = new Module(4, 8, 60, arvind, 3, 4);

    private static final SwerveDrivetrainConstants drivetrainConstants = new SwerveDrivetrainConstants()
      .withCANbusName(kCANbusName)
      .withPigeon2Id(Pigeon2.id)
      .withPigeon2Configs(Pigeon2.config);

    // Necessary
    static double kWheelRadiusInches = 1.5;
    static double kDriveGearRatio = 6.75;
    static double kSteerGearRatio = 150.0 / 7.0;
    static SwerveModule.ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;
    static SwerveModule.ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    static SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.RemoteCANcoder;
    static double kSpeedAt12VoltsMps = 4.72; 
    // TODO: Tune
    static Slot0Configs driveGains = new Slot0Configs()
      .withKP(3).withKI(0).withKD(0);
    static Slot0Configs steerGains = new Slot0Configs()
      .withKP(100).withKI(0).withKD(0.2);
    
    // For simulations
    static double kDriveInertia = 0.00001;
    static double kSteerInertia = 0.001;
    static double kSteerFrictionVoltage = 0.25;
    static double kDriveFrictionVoltage = 0.25;

    // Optional but recommended
    static double kCoupleRatio = 3; // TODO: tune. Every 1 rotation of steer results in kCoupleRatio drive turns
    static double kSlipCurrentA = 150; // TODO: Tune
    static CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
    static TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration();
    static TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()
      .withCurrentLimits(
        new CurrentLimitsConfigs()
          .withStatorCurrentLimit(80)
          .withStatorCurrentLimitEnable(true)
      );
    static boolean kSteerMotorReversed = false;

    // brotha ew
    private static final SwerveModuleConstantsFactory CONSTANTS_CREATOR = new SwerveModuleConstantsFactory()
      .withDriveMotorGearRatio(kDriveGearRatio)
      .withSteerMotorGearRatio(kSteerGearRatio)
      .withWheelRadius(kWheelRadiusInches)
      .withSlipCurrent(kSlipCurrentA)
      .withSteerMotorGains(steerGains)
      .withDriveMotorGains(driveGains)
      .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
      .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
      .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
      .withSteerInertia(kSteerInertia)
      .withDriveInertia(kDriveInertia)
      .withSteerFrictionVoltage(kSteerFrictionVoltage)
      .withDriveFrictionVoltage(kDriveFrictionVoltage)
      .withFeedbackSource(kSteerFeedbackType)
      .withCouplingGearRatio(kCoupleRatio)
      .withSteerMotorInverted(kSteerMotorReversed)
      .withDriveMotorInitialConfigs(driveInitialConfigs)
      .withSteerMotorInitialConfigs(steerInitialConfigs)
      .withCANcoderInitialConfigs(cancoderInitialConfigs);
    

      
    //public static final SwerveDriveSubsystem kSwerveDrivetrain = new SwerveDriveSubsystem(drivetrainConstants, fl, fr, bl, br);
      //TODO:Mechanical Finish this damn drivebase
      //THIS REQUIRES MECHANICAL TO FINISH THE ROBOT
  }
}
