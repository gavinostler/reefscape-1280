package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Drivetrain;


public class SwerveDriveSubsystem extends SwerveDrivetrain implements Subsystem {

    SwerveDrivetrainConstants drivetrainConstants;
    SwerveModuleConstants[] moduleConstants;

    public SwerveDriveSubsystem(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants[] modules) {
        super(drivetrainConstants, modules);
    }

    public SwerveDrivetrainConstants gSwerveDrivetrainConstants(){
        return drivetrainConstants;
    }
    public SwerveModuleConstants[] gModuleConstants(){
        return moduleConstants;
    }
    // fml (forge mod loader)
}
