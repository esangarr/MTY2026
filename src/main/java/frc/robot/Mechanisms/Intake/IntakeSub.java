package frc.robot.Mechanisms.Intake;

import com.stzteam.forgemini.io.IOSubsystem;
//Importa librerias para el motor
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class IntakeSub extends IOSubsystem{
  //Cambiar ID motores
    SparkMax motorINang = new SparkMax(1, MotorType.kBrushless); //Motor angulador
    SparkMax motorINru = new SparkMax(2, MotorType.kBrushless); //Motor ruedas

    public IntakeSub(){
        super("IntakeSubsystem");
        
    }
    //Motor angulador 
    public void Mover_motorINang(double speed){   //Mueve un motor al valor de la variable 
        motorINang.set(speed);   
    }
    public void Stop_motorINang(){  //Detiene el motor
        motorINang.stopMotor();
    }
    //Motor ruedas
    public void Mover_motorINru(double speed){   //Mueve un motor al valor de la variable 
        motorINru.set(speed);   
    }
    public void Stop_motorINru(){  //Detiene el motor
        motorINru.stopMotor();
    }

    @Override
    public void periodicLogic() {}{}

}
