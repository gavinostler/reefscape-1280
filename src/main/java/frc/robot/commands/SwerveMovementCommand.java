package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Drivetrain;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class SwerveMovementCommand extends Command {
    private final SwerveDriveSubsystem m_swerve; 
    private PhoenixPIDController m_thetaController;

    private final DoubleSupplier x_velocity_supplier;
    private final DoubleSupplier y_velocity_supplier;
    private final DoubleSupplier rot_supplier;

    private double x_velocity, y_velocity, rot;
    private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private SwerveRequest.FieldCentricFacingAngle driveAngle = new SwerveRequest.FieldCentricFacingAngle();
    private SwerveRequest request;

    private double speedMultiplier = 0.5;
    private double angularSpeedMultiplier = 0.8;

    public SwerveMovementCommand(SwerveDriveSubsystem swerve,
        DoubleSupplier x_velocity_supplier,
        DoubleSupplier y_velocity_supplier,
        DoubleSupplier rot_supplier
    ) {
        this.addRequirements(swerve);

        this.m_swerve = swerve;
        this.x_velocity_supplier = x_velocity_supplier;
        this.y_velocity_supplier = y_velocity_supplier;
        this.rot_supplier = rot_supplier;
    }

    @Override
    public void initialize() {
        m_thetaController = new PhoenixPIDController(3.2, 0, 0);
        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
        driveAngle.HeadingController = m_thetaController;
    }

    @Override
    public void execute() {
        this.x_velocity = MathUtil.applyDeadband(x_velocity_supplier.getAsDouble() * this.speedMultiplier, 0.1);
        this.y_velocity = MathUtil.applyDeadband(y_velocity_supplier.getAsDouble() * this.speedMultiplier, 0.1);
        this.rot = MathUtil.applyDeadband(rot_supplier.getAsDouble() * this.angularSpeedMultiplier, 0.1);

        SmartDashboard.putNumber("x_vel", x_velocity);
        SmartDashboard.putNumber("y_vel", y_velocity);
        SmartDashboard.putNumber("rot", rot);

        this.request = drive
            .withVelocityX(this.x_velocity * Drivetrain.MAX_VELOCITY)
            .withVelocityY(this.y_velocity * Drivetrain.MAX_VELOCITY)
            .withRotationalRate(this.rot * Drivetrain.MAX_TURN_RATE);

        this.m_swerve.setControl(request);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
