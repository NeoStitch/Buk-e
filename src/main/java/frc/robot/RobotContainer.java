// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;




import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
//import edu.wpi.first.math.proto.Trajectory;
//import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.GanchoCom;
import frc.robot.commands.IntakeCom;
import frc.robot.commands.ShooterCom;
import frc.robot.commands.SwerveController;
import frc.robot.subsystems.GanchoSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ShooterSub;


public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final XboxController control1 = new XboxController(1);
  public final XboxController control = new XboxController(0);
  private IntakeSub intakeSub = new IntakeSub();
  private ShooterSub shooterSub = new ShooterSub();
  private GanchoSub ganchosub = new GanchoSub();
  private final JoystickButton LeftBumper = new JoystickButton(control, XboxController.Button.kLeftBumper.value);
  private final JoystickButton RightBumper = new JoystickButton(control, XboxController.Button.kRightBumper.value);

  private final JoystickButton a = new JoystickButton(control, XboxController.Button.kA.value);
  private final JoystickButton b = new JoystickButton(control, XboxController.Button.kB.value);
  private final JoystickButton x = new JoystickButton(control, XboxController.Button.kX.value);
  private final JoystickButton y = new JoystickButton(control, XboxController.Button.kY.value);
private final POVButton DpadRight = new POVButton(control, 90);
private final POVButton Dpadleft = new POVButton(control, 270);
private final POVButton Dpadup = new POVButton(control, 0);
private final POVButton Dpaddown = new POVButton(control, 180);





  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(
    new SwerveController(
    swerveSubsystem,
    ()->-control.getLeftY(), 
    ()->-control.getLeftX() , 
    ()-> -control.getRightTriggerAxis()+control.getLeftTriggerAxis(), 
    ()-> control.getRightStickButton()));
  configureButtonBindings();
  }

  private void configureButtonBindings(){
   // zeroGyro.onTrue(new InstantCommand(()->swerveSubsystem.zeroHeading()));

   //Intake
      LeftBumper.whileTrue(new IntakeCom(intakeSub, 0.5, 0));
      RightBumper.whileTrue(new IntakeCom(intakeSub, -0.5, 0));
      b.whileTrue(new IntakeCom(intakeSub, 0, .95));
      a.whileTrue(new IntakeCom(intakeSub, 0, -.95));

      //Shooter
      x.whileTrue(new ShooterCom(shooterSub, 0, .40));
      y.whileTrue(new ShooterCom(shooterSub, 0, .90));
      Dpadup.whileTrue(new ShooterCom(shooterSub, -.70, 0));
      Dpaddown.whileTrue(new ShooterCom(shooterSub, .70, 0));

      // Gancho
      DpadRight.whileTrue(new GanchoCom(ganchosub, -0.5, 0.5));
      Dpadleft.whileTrue(new GanchoCom(ganchosub, 0.5, -0.5));      
  }  
  public Command getAutonomousCommand() {

    /*TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
       AutoConstants.kMaxAccelerationMetersPerSecondSquared)
             .setKinematics(DriveConstants.kDriveKinematics);

      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)), 
      List<Translation2d>.of(
            new Translation2d(1, 0),
            new Translation2d(1, -1)        
      ),
      new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
      trajectoryConfig
      );

      PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);  
      PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0); 
      ProfiledPIDCommand thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraits);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
           trajectory, swerveSubsystem::getPose, DriveConstants.kDriveKinematics, xController,yController, thetaController, swerveSubsystem::setModulesStates, swerveSubsystem);

          return new SequentialCommandGroup(
            new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
            swerveControllerCommand,
            new InstantCommand(() -> swerveSubsystem.stopModules())
          ); */
    return new SequentialCommandGroup( //
      new IntakeCom(intakeSub, 0, 0.4), //
      
      new ShooterCom(shooterSub, 0, 0.5)
       //
    ); 
  } 
  
}
