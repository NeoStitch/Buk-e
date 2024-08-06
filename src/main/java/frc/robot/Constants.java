package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1/6.75;
    public static final double kRotMotorGearRatio = 7/150.0;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio*Math.PI*kWheelDiameterMeters;
    public static final double kRotEncoderRot2Rad = kRotMotorGearRatio*2*Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter/60;
    public static final double kRotEncoderRPM2RadPerSec = kRotEncoderRot2Rad/60;
    public static final double kP_rot = 0.5;

    public static final class DriveConstants{
        public static final double kTrackWidth = Units.inchesToMeters(21.5);
        public static final double kWheelBase = Units.inchesToMeters(21.5);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase/2, -kTrackWidth/2),
            new Translation2d(kWheelBase/2, kTrackWidth/2),
            new Translation2d(-kWheelBase/2, -kTrackWidth/2),
            new Translation2d(-kWheelBase/2, kTrackWidth/2));
        
            public static final int kA_DriveMotorPort = 7 ;
            public static final int kA_RotMotorPort = 6;
            public static final int kA_AbsEncoderPort = 8;
            
            public static final int kB_DriveMotorPort = 4 ;
            public static final int kB_RotMotorPort = 3;
            public static final int kB_AbsEncoderPort = 5;

            public static final int kC_DriveMotorPort = 13 ;
            public static final int kC_RotMotorPort = 12;
            public static final int kC_AbsEncoderPort = 14;

            public static final int kD_DriveMotorPort = 10 ;
            public static final int kD_RotMotorPort = 9;
            public static final int kD_AbsEncoderPort = 11;

            public static final boolean kA_RotEncoderReversed = true;
            public static final boolean kC_RotEncoderReversed = true;
            public static final boolean kB_RotEncoderReversed = true;
            public static final boolean kD_RotEncoderReversed = true;

            public static final boolean kA_DriveEncoderReversed = true;
            public static final boolean kC_DriveEncoderReversed = true;
            public static final boolean kB_DriveEncoderReversed = true;
            public static final boolean kD_DriveEncoderReversed = true;

            public static final boolean kA_AbsoluteEncoderReversed = false;
            public static final boolean kC_AbsoluteEncoderReversed = false;
            public static final boolean kB_AbsoluteEncoderReversed = false;
            public static final boolean kD_AbsoluteEncoderReversed = false;

            public static final double kA_DriveAbsoluteEncoderOffsetRad = 1.48;
            public static final double kB_DriveAbsoluteEncoderOffsetRad = 2.81;
            public static final double kC_DriveAbsoluteEncoderOffsetRad = 3.24;
            public static final double kD_DriveAbsoluteEncoderOffsetRad = 2.17;

            public static final double kPhysicalMaxSpeedMetersPerSecond = 4;
            public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

            public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 1.5;
            public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 1.5;
            public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 6;
            public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 6;
    }
    public static final class AutoConstants{
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond /10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraits = 
         new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);

    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.05;
    }
    public class IntakeConstants{
        public static int Intake_Recoge_port = 21;
        public static int Intake_mueve_port = 25; 
    }
    public class ShooterConstants{
        public static int Shooter_lanza1_port = 22;
        public static int Shooter_lanza2_port = 23;
        public static int Shooter_mueve_port = 24;
    }
    public class GanchoConstants {
        public static int Gancho_Izquierdo_port = 26;
        public static int Gancho_Derecho_port = 27;
    
        
    }

}
