package frc.robot.Mechanisms.Intake;

import edu.wpi.first.wpilibj2.command.Command;



public class Mover_motoresIN extends Command{ //Crea el comando con herencia 
    private IntakeSub intake; //Define variables
    private double speed;

    public void Mover_motorINang(IntakeSub intake, double speed){
        this.intake = intake; //Guardan los valores para poderlos usar en el execute y end (Variables de instancia)
        this.speed = speed;

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute(){ //Lo que hace mientras se ejecuta el código
        intake.Mover_motorINang(speed); //Mueve motor a lo que le diga la variable speed
        intake.Mover_motorINru(speed);
    
    }

    @Override
    public void end(boolean interrupted) { //Lo que hace cuando acaba el código
      intake.Stop_motorINang(); //Detiene motor
      intake.Stop_motorINru();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
