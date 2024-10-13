package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import edu.wpi.first.wpilibj2.command.Subsystem;


public class SwerveDriveSubsystem extends SwerveDrivetrain implements Subsystem {

    public SwerveDriveSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants[] modules) {
        super(driveTrainConstants, modules);
        
    }    
    // fml (forge mod loader)
}
