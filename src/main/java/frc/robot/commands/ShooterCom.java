// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;

public class ShooterCom extends Command {
  private final ShooterSub shooterSub;
  private final double aim;
  private final double Lanza;

  public ShooterCom(ShooterSub shooterSub, double aim, double Lanza) {
    this.shooterSub = shooterSub;
    this.aim = aim;
    this.Lanza = Lanza;
    addRequirements(shooterSub);

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shooterSub.SetShoot(Lanza,aim);
  }

  @Override
  public void end(boolean interrupted) {
    shooterSub.SetShoot(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
