// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final String kCANbusName = "rio";
  public static final boolean kEnableSysId = false;

  // public static String kCANivore = "RENivore";

  public static class Driver {
    public static final int kDriverControllerPort = 0;
  }

  public static class Operator {
    public static final int kOperatorControllerPort = 1;
  }

  public static class Pigeon2 {
    static final int id = 26; // TODO: set can id
    static final Pigeon2Configuration config = null; // idk what it do
  }

  public static class Lights {
    public static final int id = 27; // CAN ID for RGB
    public static final double brightnessScalar = 1.0; // Brightness for Color
    public static final int leds = 150;
    public static final Color8Bit initialColor = new Color8Bit(199, 21, 133);
  }

  public static class Elevator {
    public static final double TOP_HEIGHT = 1.0;
    public static final double SHOOT_HEIGHT = 0.92; // TODO
    public static final double L2_HEIGHT = 0.70;
    public static final double GROUND_INTAKE_HEIGHT = 0.527;
    public static final double L1_HEIGHT = 0.427;
    public static final double BOTTOM_HEIGHT = 0.0;
    public static final double HEIGHT_TOLERANCE = 0.03;

    public static final int motorId = 18;
    public static final double CURRENT_LIMIT = 80.0;
    public static final double FF_TERM =
        0.5; // Volts to add as feedforward to account for gravity etc

    public static final TalonFXConfiguration elevatorConfigs = new TalonFXConfiguration();

    static {
      elevatorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
      elevatorConfigs.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT;
      elevatorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      elevatorConfigs.MotorOutput.Inverted =
          InvertedValue.Clockwise_Positive; // positive should make it go up
      // Assuming CANcoder is after gear reduction, and encoder is zeroed at horizontal
      elevatorConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;

      // TODO: merge kG with feedforward?
      elevatorConfigs.Slot0.kG = -0.8; // TODO: tune
      elevatorConfigs.Slot0.kS = 0.0;
      elevatorConfigs.Slot0.kV = 15.0;
      elevatorConfigs.Slot0.kA = 0.0;
      elevatorConfigs.Slot0.kP = 40.0;
      elevatorConfigs.Slot0.kI = 0.0;
      elevatorConfigs.Slot0.kD = 2.0;
      elevatorConfigs.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
      // https://www.chiefdelphi.com/t/motion-magic-help-ctre/483319/2
      elevatorConfigs.MotionMagic.MotionMagicCruiseVelocity = 2.0; // Target cruise velocity in rps
      elevatorConfigs.MotionMagic.MotionMagicAcceleration = 4.0; // Target acceleration in rps/s
      elevatorConfigs.MotionMagic.MotionMagicJerk = 40.0; // Target jerk in rps/(s^2)
      elevatorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
      elevatorConfigs.Feedback.SensorToMechanismRatio = 135.0;
      elevatorConfigs.Feedback.RotorToSensorRatio = 1.0; // note: does nothing
    }
  }

  public static class GroundIntake {
    public static final double UP_ANGLE = 0.22;
    public static final double DOWN_ANGLE = 0.084;

    public static final int encoderChannelA = 3;
    public static final int encoderChannelB = 4;
    public static final double DISTANCE_PER_PULSE = 0.000531;
    public static final double ENCODER_OFFSET = 0.25;
    public static final double ANGLE_TOLERANCE = 0.01;
    public static final boolean REVERSE_ENCODER = true;

    public static final int intakeId = 19;
    public static final int INTAKE_CURRENT_LIMIT = 20;

    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

    static {
      intakeConfig.idleMode(IdleMode.kBrake);
      intakeConfig.smartCurrentLimit(INTAKE_CURRENT_LIMIT);
    }

    public static final PIDController intakePID = new PIDController(20.0, 0.0, 3.0);
    public static final ArmFeedforward intakeFf = new ArmFeedforward(0.0, 1.0, 0.0, 0.0);

    static {
      intakePID.setTolerance(ANGLE_TOLERANCE);
    }

    public static final int pulleyId = 20;
    public static final double PULLEY_CURRENT_LIMIT = 50.0;
    public static final double PULLEY_VOLTAGE = 5.0;

    public static final TalonFXConfiguration pulleyConfigs = new TalonFXConfiguration();

    static {
      pulleyConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
      pulleyConfigs.CurrentLimits.StatorCurrentLimit = GroundIntake.PULLEY_CURRENT_LIMIT;
    }
  }

  public static class Shooter {
    public static final double ARM_MIN_ANGLE = -0.25;
    public static final double ARM_MAX_ANGLE = 0.225;

    public static final double STOW_ANGLE = 0.22;
    public static final double SHOOT_ANGLE = 0.16;
    public static final double REEF_INTAKE_ANGLE = 0.055;
    public static final double GROUND_INTAKE_ANGLE = -0.151;
    public static final double DOWN_ANGLE = -0.26;
    public static final double ANGLE_TOLERANCE = 0.01;

    public static final int rightShooterId = 12;
    public static final int leftShooterId = 14;
    public static final double SHOOTER_SHOOT_TARGET_RPS = 22.0;
    public static final double SHOOTER_INTAKE_TARGET_RPS = -15.0;
    public static final double SHOOTER_PROCESSOR_TARGET_RPS = 17.0; // TODO: set processor speed
    public static final double SHOOTER_GEAR_REDUCTION = 1.0;
    public static final double SHOOTER_CURRENT_LIMIT = 80.0;

    public static final int rightFeedId = 13;
    public static final int leftFeedId = 15;
    public static final double FEED_IN_SPEED = -0.5;
    public static final double FEED_OUT_SPEED = 0.4;
    public static final int FEED_CURRENT_LIMIT = 40;

    public static final int armId = 16;
    public static final int armEncoderId = 57;
    public static final double ARM_CURRENT_LIMIT = 80.0;

    public static final TalonFXConfiguration shooterConfigs = new TalonFXConfiguration();

    static {
      shooterConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
      shooterConfigs.CurrentLimits.StatorCurrentLimit = SHOOTER_CURRENT_LIMIT;
      shooterConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      shooterConfigs.MotorOutput.Inverted =
          InvertedValue.CounterClockwise_Positive; // right is primary, positive is out
      // TODO: tune gains
      shooterConfigs.Slot0.kS = 0.0; // Add kS V output to overcome static friction
      shooterConfigs.Slot0.kV = 0.1; // A velocity target of 1 rps results in kV V output
      shooterConfigs.Slot0.kA = 0.2; // An acceleration of 1 rps/s requires kA V output
      shooterConfigs.Slot0.kP = 0.15; // An error of 1 rps results in kP V output
      shooterConfigs.Slot0.kI = 0.0; // integrated error (0: no output for error)
      shooterConfigs.Slot0.kD = 0.0; // error derivative
      shooterConfigs.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    }

    public static final SparkMaxConfig followerFeedConfig = new SparkMaxConfig();
    public static final SparkMaxConfig leaderFeedConfig = new SparkMaxConfig();

    static {
      final SparkMaxConfig feedConfig = new SparkMaxConfig();
      final ClosedLoopConfig pidConfig = new ClosedLoopConfig();
      pidConfig.pidf(0.0, 0.0, 0.0, 0.0); // TODO: tune for braking
      feedConfig.apply(pidConfig);
      feedConfig.idleMode(IdleMode.kBrake); // note: brake isn't actually working
      feedConfig.smartCurrentLimit(FEED_CURRENT_LIMIT);
      leaderFeedConfig.apply(feedConfig).inverted(true);
      followerFeedConfig
          .apply(feedConfig)
          .follow(rightFeedId, true); // follow in opposite direction
    }

    public static final TalonFXConfiguration armConfigs = new TalonFXConfiguration();

    static {
      armConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
      armConfigs.CurrentLimits.StatorCurrentLimit = ARM_CURRENT_LIMIT;
      armConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      armConfigs.MotorOutput.Inverted =
          InvertedValue.CounterClockwise_Positive; // positive should make it go up
      // Assuming CANcoder is after gear reduction, and encoder is zeroed at horizontal
      armConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
      armConfigs.Slot0.kG = 0.1;
      armConfigs.Slot0.kS = 0.0;
      armConfigs.Slot0.kV = 0.0;
      armConfigs.Slot0.kA = 0.0;
      armConfigs.Slot0.kP = 30.0;
      armConfigs.Slot0.kI = 0.0;
      armConfigs.Slot0.kD = 1.0;
      armConfigs.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
      // https://www.chiefdelphi.com/t/motion-magic-help-ctre/483319/2
      armConfigs.MotionMagic.MotionMagicCruiseVelocity = 0.2; // Target cruise velocity in rps
      armConfigs.MotionMagic.MotionMagicAcceleration = 0.4; // Target acceleration in rps/s
      armConfigs.MotionMagic.MotionMagicJerk = 0.8; // Target jerk in rps/(s^2)
      armConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
      armConfigs.Feedback.FeedbackRemoteSensorID = armEncoderId;
      armConfigs.Feedback.SensorToMechanismRatio = 1.0;
    }
  }

  public static class Vision {
    // Reef
    public static int[] reefIds = new int[] {
      6,
      7,
      8,
      9,
      10,
      11,
      17,
      18,
      19,
      20,
      21,
      22,
    };
    
    public static HashMap<Integer, Boolean> reefAlgaeMap = new HashMap<>();
    static {
      for (int i = 6; i < 12; i++) {
        reefAlgaeMap.put(i, i % 2 != 0);
      }
      for (int i = 17; i < 23; i++) {
        reefAlgaeMap.put(i, i % 2 == 0);
      }
    }
    
    public static Transform2d reefAlign = new Transform2d(0.1, 0.0, new Rotation2d());
    public static Transform2d reefAlignFar = new Transform2d(0.7, 0.0, new Rotation2d());
    
    public static double reefMaxVelocity = 1.5;
    public static double reefInMaxVelocity = 1;
    public static double reefMaxAcceleration = 2;
    
    public static double reefMaxRotationalRate = Units.degreesToRadians(3000);
    public static double reefMaxAccelerationRotationalRate = Units.degreesToRadians(2000);
    
    // Barge
    public static HashMap<Alliance, Integer> bargeAllianceMap = new HashMap<>();
    static {
      bargeAllianceMap.put(Alliance.Blue, 14);
      bargeAllianceMap.put(Alliance.Red, 5);
    }
    
    public static Transform2d bargeAlign = new Transform2d(0.8128, 0, new Rotation2d()); // calculated from CAD
    
    public static double bargeMaxVelocity = 1;
    public static double bargeMaxAcceleration = 1;
    
    public static double bargeLength = 3.7; // in meters, calculated from CAD
    
    // Processor
    public static int[] processorIds = new int[] {16, 3};
    
  }
}
