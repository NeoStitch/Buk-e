// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GanchoSub;

public class GanchoCom extends Command {
  private final GanchoSub ganchoSub;
  private final  double rotLeft;
  private final double rotRight;

  public GanchoCom(GanchoSub ganchoSub, double rotLeft, double rotRight) {
    this.ganchoSub = ganchoSub;
    this.rotLeft = rotLeft;
    this.rotRight = rotRight;
    addRequirements(ganchoSub);
  }


  @Override
  public void initialize() {

  }

  
  @Override
  public void execute() {
    ganchoSub.setLeft(rotLeft,rotRight);

  }

  
  @Override
  public void end(boolean interrupted) {
      ganchoSub.setLeft(0,0);

  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}
