
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GanchoSub;

public class GanchoPIDCom extends Command {
    private final GanchoSub ganchoSub;
    private final PIDController pidController;
  public GanchoPIDCom(GanchoSub ganchoSub, double setpoint) {
    this.ganchoSub = ganchoSub;
    this.pidController = new PIDController(3, 0, 0.8);
    pidController.setSetpoint(setpoint);
    addRequirements(ganchoSub);
  }

  @Override
  public void initialize() {
    pidController.reset();
  }

  @Override
  public void execute() {
    double speed1 = pidController.calculate(ganchoSub.getEncoderMetersDer());
    double speed2 = pidController.calculate(ganchoSub.getEncoderMetersIzq());
    ganchoSub.setLeft(speed2, speed1);

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
