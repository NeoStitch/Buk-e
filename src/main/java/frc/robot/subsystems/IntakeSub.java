package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSub extends SubsystemBase{
    private CANSparkMax Intake_Recoge = new CANSparkMax(IntakeConstants.Intake_Recoge_port, MotorType.kBrushed);
    private CANSparkMax Intake_mueve = new CANSparkMax(IntakeConstants.Intake_mueve_port, MotorType.kBrushless);

    public IntakeSub(){
        Intake_Recoge.set(0);
        Intake_mueve.set(0);
        Intake_mueve.setIdleMode(IdleMode.kBrake);
    }
    @Override
    public void periodic(){
    }
    public void SetIn(double Recoger, double apuntar){
        Intake_Recoge.set(Recoger);
        Intake_mueve.set(apuntar);
    }
}
