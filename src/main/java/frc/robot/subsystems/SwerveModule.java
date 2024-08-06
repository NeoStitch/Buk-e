package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax rotMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder rotEncoder;

    private final PIDController rotPidController;

    private final CANcoder cancoder;

    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorID, int rotMotorID, boolean driveMotorReversed,
    boolean rotMotorReversed, int CANcoderID, double absoluteEncoderOffset,
            boolean absoluteEncoderReversed){
                this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
                this.absoluteEncoderReversed = absoluteEncoderReversed;
                cancoder = new CANcoder(CANcoderID);

                driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
                rotMotor = new CANSparkMax(rotMotorID, MotorType.kBrushless);

                driveMotor.setInverted(driveMotorReversed);
                rotMotor.setInverted(rotMotorReversed);

                driveEncoder = driveMotor.getEncoder();
                rotEncoder = rotMotor.getEncoder();

                driveEncoder.setPositionConversionFactor(Constants.kDriveEncoderRot2Meter);
                driveEncoder.setVelocityConversionFactor(Constants.kDriveEncoderRPM2MeterPerSec);
                rotEncoder.setPositionConversionFactor(Constants.kRotEncoderRot2Rad);
                rotEncoder.setVelocityConversionFactor(Constants.kRotEncoderRPM2RadPerSec);

                rotPidController = new PIDController(Constants.kP_rot, 0, 0);
                rotPidController.enableContinuousInput(-Math.PI, Math.PI);

                resetEncoder();
    }
 


    public double getDrivePosition(){
        return driveEncoder.getPosition();
    }

    public double getRotPosition(){
        return rotEncoder.getPosition();
    }

    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }

    public double getRotVelocity(){
        return rotEncoder.getVelocity();
    }

    public double getAbsEncoder(){
        StatusSignal<Double> positionSupplier = cancoder.getAbsolutePosition();
        double angle = positionSupplier.getValueAsDouble();
        angle *= 2.0*Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle*(absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoder(){
        driveEncoder.setPosition(0);
        rotEncoder.setPosition(getAbsEncoder());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(),new Rotation2d(getRotPosition()));
    }
        public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(
        driveEncoder.getPosition(), new Rotation2d(rotEncoder.getPosition()));
}

    public void setDesiredState(SwerveModuleState state){
        if(Math.abs(state.speedMetersPerSecond)<0.001){
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond/DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        rotMotor.set(rotPidController.calculate(getRotPosition(), state.angle.getRadians()));
        
        SmartDashboard.putString("Swerve["+cancoder.getDeviceID()+"] state", state.toString());
    }

    public void stop(){
        driveMotor.set(0);
        rotMotor.set(0);
    }
 
}