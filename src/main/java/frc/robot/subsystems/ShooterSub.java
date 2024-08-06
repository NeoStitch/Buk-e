package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSub extends SubsystemBase{
    private CANSparkMax Shooter_lanza1 = new CANSparkMax(ShooterConstants.Shooter_lanza1_port, MotorType.kBrushless);
    private CANSparkMax Shooter_lanza2 = new CANSparkMax(ShooterConstants.Shooter_lanza2_port, MotorType.kBrushless);
    private CANSparkMax Shooter_mueve = new CANSparkMax(ShooterConstants.Shooter_mueve_port, MotorType.kBrushless);

    public ShooterSub(){
        Shooter_lanza1.set(0);
        Shooter_mueve.set(0);
            Shooter_mueve.setIdleMode(IdleMode.kBrake);
            Shooter_mueve.set(0);

    }

    public void SetShoot(double Lanza, double aim){
    Shooter_lanza1.set(Lanza);
    Shooter_lanza2.set(-Lanza);
    Shooter_mueve.set(aim);
    }
}