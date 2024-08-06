package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GanchoConstants;



public class GanchoSub extends SubsystemBase{
    private final CANSparkMax Gancho_Izquierdo = new CANSparkMax(GanchoConstants.Gancho_Izquierdo_port, MotorType.kBrushless); 
    private final CANSparkMax Gancho_Derecho = new CANSparkMax(GanchoConstants.Gancho_Derecho_port, MotorType.kBrushless); 
    private final double kEncoderTick2Meter = 1.0/4096.0 * 0.1 * Math.PI;
    private RelativeEncoder Encoder_ganchoIzq = Gancho_Izquierdo.getEncoder();
    private RelativeEncoder Encoder_ganchoDer = Gancho_Derecho.getEncoder();

   public  GanchoSub() {
    Gancho_Derecho.setIdleMode(IdleMode.kBrake);
    Gancho_Izquierdo.setIdleMode(IdleMode.kBrake);
 }

 public double getEncoderMetersDer(){
   return Encoder_ganchoDer.getPosition() * kEncoderTick2Meter;  
 }
public double getEncoderMetersIzq(){
   return Encoder_ganchoIzq.getPosition() * kEncoderTick2Meter;
}


    @Override
    public void periodic(){
   
    System.out.println("Puro Amp");
    }
 
 public void setLeft(double rotLeft, double rotRight){
   Gancho_Izquierdo.set(rotLeft);
   Gancho_Derecho.set(rotRight);
 }


  

// antes de usar revisar si es positivo o negativo
 
}
