package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
public class SwerveSubsystem extends SubsystemBase{

    private final SwerveModule ModuleA = new SwerveModule(
        DriveConstants.kA_DriveMotorPort,
        DriveConstants.kA_RotMotorPort,
        DriveConstants.kA_DriveEncoderReversed,
        DriveConstants.kA_RotEncoderReversed,
        DriveConstants.kA_AbsEncoderPort,
        DriveConstants.kA_DriveAbsoluteEncoderOffsetRad,
        DriveConstants.kA_AbsoluteEncoderReversed);

    private final SwerveModule ModuleB = new SwerveModule(
        DriveConstants.kB_DriveMotorPort,
        DriveConstants.kB_RotMotorPort,
        DriveConstants.kB_DriveEncoderReversed,
        DriveConstants.kB_RotEncoderReversed,
        DriveConstants.kB_AbsEncoderPort,
        DriveConstants.kB_DriveAbsoluteEncoderOffsetRad,
        DriveConstants.kB_AbsoluteEncoderReversed);

    private final SwerveModule ModuleC = new SwerveModule(
        DriveConstants.kC_DriveMotorPort,
        DriveConstants.kC_RotMotorPort,
        DriveConstants.kC_DriveEncoderReversed,
        DriveConstants.kC_RotEncoderReversed,
        DriveConstants.kC_AbsEncoderPort,
        DriveConstants.kC_DriveAbsoluteEncoderOffsetRad,
        DriveConstants.kC_AbsoluteEncoderReversed);

    private final SwerveModule ModuleD = new SwerveModule(
        DriveConstants.kD_DriveMotorPort,
        DriveConstants.kD_RotMotorPort,
        DriveConstants.kD_DriveEncoderReversed,
        DriveConstants.kD_RotEncoderReversed,
        DriveConstants.kD_AbsEncoderPort,
        DriveConstants.kD_DriveAbsoluteEncoderOffsetRad,
        DriveConstants.kD_AbsoluteEncoderReversed);

    private Pigeon2 gyro = new Pigeon2(15);
    
 ChassisSpeeds speeds = new ChassisSpeeds(1.0,3.0,1.5);

    public SwerveSubsystem(){
        new Thread(()-> {
            try{
                Thread.sleep(1000);
                zeroHeading();
            }catch (Exception e){
            }
        }).start();
    }
    
    public void zeroHeading(){
        gyro.reset();
    }

    public double getHeading(){
        return Math.IEEEremainder(gyro.getAngle(),360);
    }

    public Rotation2d getRotation2D(){
        return Rotation2d.fromDegrees(getHeading());      
    }
    //Translation2d ModuleA = new Translation2d(0.381, 0.381);
  SwerveDriveOdometry m_Odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getRotation2D(), new SwerveModulePosition[]{
    ModuleA.getPosition(),
    ModuleB.getPosition(),
    ModuleC.getPosition(),
    ModuleD.getPosition()
  }, new Pose2d(5.0, 13.5, new Rotation2d()));

    public Pose2d getPose(){
        return m_Odometry.getPoseMeters();
    } 
   public void resetOdometry(Pose2d pose){
        m_Odometry.resetPosition(getRotation2D(),new SwerveModulePosition[]{
            ModuleA.getPosition(), ModuleB.getPosition(),
            ModuleC.getPosition(), ModuleD.getPosition()}, pose);
    }


  
    @Override
    public void periodic() {
        var gyroAngle = getRotation2D();
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        
      m_Odometry.update(gyroAngle, 
        new SwerveModulePosition[]{
            ModuleA.getPosition(), ModuleB.getPosition(),
            ModuleC.getPosition(), ModuleD.getPosition()
        });
    }


    public void stopModules(){
        ModuleA.stop();
        ModuleB.stop();
        ModuleC.stop();
        ModuleD.stop();
    }

    public void setModulesStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        ModuleA.setDesiredState(desiredStates[0]);
        ModuleB.setDesiredState(desiredStates[1]);
        ModuleC.setDesiredState(desiredStates[2]);
        ModuleD.setDesiredState(desiredStates[3]);
    }

}
