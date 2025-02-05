package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Drivetrain;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class SwerveMovementCommand extends Command {
    private final SwerveDriveSubsystem m_swerve; 
    private PhoenixPIDController m_thetaController;

    private final DoubleSupplier x_supplier;
    private final DoubleSupplier y_supplier;
    private final DoubleSupplier rot_supplier;

    private double x_val, y_val, rot_val;
    private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private SwerveRequest.FieldCentricFacingAngle driveAngle = new SwerveRequest.FieldCentricFacingAngle();
    private SwerveRequest request;

    private double speedMultiplier = 0.5;
    private double angularSpeedMultiplier = 0.8;
    private double driveDeadband = 0.1;

    public SwerveMovementCommand(SwerveDriveSubsystem swerve,
        DoubleSupplier x_supplier,
        DoubleSupplier y_supplier,
        DoubleSupplier rot_supplier
    ) {
        this.addRequirements(swerve);

        this.m_swerve = swerve;
        this.x_supplier = x_supplier;
        this.y_supplier = y_supplier;
        this.rot_supplier = rot_supplier;
    }

    @Override
    public void initialize() {
        m_thetaController = new PhoenixPIDController(3.2, 0, 0);
        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
        driveAngle.HeadingController = m_thetaController;
    }

    public void handleInput() {
        this.rot_val = MathUtil.applyDeadband(rot_supplier.getAsDouble() * this.angularSpeedMultiplier, 0.1);
        double x = x_supplier.getAsDouble();
        double y = y_supplier.getAsDouble(); 

        // deadband drive vector 
        double magnitude = MathUtil.applyDeadband(Math.sqrt(x*x + y*y), driveDeadband);
        if (magnitude == 0.0) {
            this.x_val = 0.0;
            this.y_val = 0.0;
        } else {
            this.x_val = x / Math.abs(x) * MathUtil.interpolate(0, this.speedMultiplier, Math.abs(x));
            this.y_val = y / Math.abs(y) * MathUtil.interpolate(0, this.speedMultiplier, Math.abs(y));
        }
    }

    @Override
    public void execute() {
        handleInput();

        this.request = drive
            .withVelocityX(this.x_val * Drivetrain.MAX_DRIVE_VOLTAGE)
            .withVelocityY(this.y_val * Drivetrain.MAX_DRIVE_VOLTAGE)
            .withRotationalRate(this.rot_val * Drivetrain.MAX_TURN_VOLTAGE);

        this.m_swerve.setControl(request);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
