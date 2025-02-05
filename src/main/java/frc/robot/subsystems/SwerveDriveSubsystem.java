package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Drivetrain;

public class SwerveDriveSubsystem extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
  public SwerveDriveSubsystem() {
    super(
      TalonFX::new, TalonFX::new, CANcoder::new,
      Drivetrain.DrivetrainConstants, 
      Drivetrain.moduleConstants
    );
  }

  private SwerveRequest.ApplyRobotSpeeds drive = new SwerveRequest.ApplyRobotSpeeds();

  public Pose2d getPose(){
    return this.getState().Pose;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return this.getKinematics().toChassisSpeeds(this.getState().ModuleStates);       
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    this.setControl(drive.withSpeeds(speeds));
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return this.run(() -> this.setControl(requestSupplier.get()));
  }
}
