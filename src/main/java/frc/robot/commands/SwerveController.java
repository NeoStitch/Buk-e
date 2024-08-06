package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveController extends Command{
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, rotSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, rotLimiter;

    public SwerveController(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction, 
    Supplier<Double> ySpdFunction, Supplier<Double> rotSpdFunction, Supplier<Boolean> fieldOrientedFunction){
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.rotSpdFunction = rotSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.rotLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double rotSpeed = rotSpdFunction.get();

        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed: 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed: 0.0;
        rotSpeed = Math.abs(rotSpeed) > OIConstants.kDeadband ? rotSpeed: 0.0;

        xSpeed = xLimiter.calculate(xSpeed)*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed)*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        rotSpeed = rotLimiter.calculate(rotSpeed)*DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        ChassisSpeeds chassisSpeeds;

        if(fieldOrientedFunction.get()){
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rotSpeed,swerveSubsystem.getRotation2D());
        }else{
            chassisSpeeds = new ChassisSpeeds(xSpeed,ySpeed,rotSpeed);
        }

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        swerveSubsystem.setModulesStates(moduleStates);

    }

    
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }    
    
}
