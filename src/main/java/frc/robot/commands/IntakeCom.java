// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSub;

public class IntakeCom extends Command {
  private final IntakeSub intakeSub;
  private final double apuntar;
  private final double Recoger;

  public IntakeCom(IntakeSub intakeSub, double apuntar, double Recoger) {
  this.intakeSub = intakeSub;
  this.apuntar = apuntar;
  this.Recoger = Recoger;
  addRequirements(intakeSub);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeSub.SetIn(Recoger, apuntar);
  }

  @Override
  public void end(boolean interrupted) {
            intakeSub.SetIn(0, 0);

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
