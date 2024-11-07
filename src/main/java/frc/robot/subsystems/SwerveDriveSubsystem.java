package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.PointWheelsAt;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drivetrain;

public class SwerveDriveSubsystem extends SubsystemBase {
  public SwerveDriveSubsystem() {}

  final PointWheelsAt m_pointWheelsAtRequest = new PointWheelsAt();
  final RobotCentric m_robotCentricRequest = new RobotCentric();
  private double angle = 0.0;
  @Override
  public void periodic() {
    angle += 5.0;
    Rotation2d rotation = new Rotation2d(angle);
    Drivetrain.swerveDrivetrain.setControl(m_pointWheelsAtRequest.withModuleDirection(rotation));
    Drivetrain.swerveDrivetrain.setControl(m_robotCentricRequest.withVelocityX(0.2));
  }
}
