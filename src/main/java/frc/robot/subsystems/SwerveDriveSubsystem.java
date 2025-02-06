package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Drivetrain;

public class SwerveDriveSubsystem extends LegacySwerveDrivetrain implements Subsystem {
  public SwerveDriveSubsystem() {
    super(
      Drivetrain.DrivetrainConstants, 
      Drivetrain.moduleConstants
    );
  }

  private LegacySwerveRequest.ApplyChassisSpeeds drive = new LegacySwerveRequest.ApplyChassisSpeeds();

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

  public Command applyRequest(Supplier<LegacySwerveRequest> requestSupplier) {
    return this.run(() -> this.setControl(requestSupplier.get()));
  }

  public void periodic() {
    for(int i=0; i<4; i++) {
      SmartDashboard.putData("encoders ("+i+")", this.getModule(i).getCANcoder());
    }
  }
}
