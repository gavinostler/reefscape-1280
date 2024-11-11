package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Drivetrain;

public class SwerveDriveSubsystem extends SwerveDrivetrain implements Subsystem {
  public SwerveDriveSubsystem() {
    super(
      Drivetrain.DrivetrainConstants, 
      Drivetrain.moduleConstants
    );
  }

  private SwerveRequest.ApplyChassisSpeeds drive = new SwerveRequest.ApplyChassisSpeeds();

  public Pose2d getPose(){
    return this.getState().Pose;
  }

  public void resetPose(Pose2d newPose){
    this.seedFieldRelative(newPose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return this.m_kinematics.toChassisSpeeds(this.getState().ModuleStates);       
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    this.setControl(drive.withSpeeds(speeds));
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return this.run(() -> this.setControl(requestSupplier.get()));
  }
}
