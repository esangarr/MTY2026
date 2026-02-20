package frc.robot.Mechanisms;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class MechanismsConstants {
    
    //Constantes del Index
    public static class IndexConstants {
        
        // IDs de los 2 motores, rodillos e index
        public static final int ROLLERS_MOTOR_CAN_ID = 0;
        public static final int INDEX_MOTOR_CAN_ID = 1;
        
       // Esto es por si el motor gira al revés, en ese caso cambiar a true
        public static final boolean ROLLERS_INVERTED = false;
        public static final boolean INDEX_INVERTED = false;
        
        //Velocidad de los 2 motores
        public static final double INDEXER_SPEED = 0.75;  // 75% speed
        
       //Sensor, si se tiene uno ._.
        public static final boolean HAS_GAME_PIECE_SENSOR = false;
        public static final int GAME_PIECE_SENSOR_DIO_PORT = 0;
        
        // Telemetría
        public static final double TELEMETRY_UPDATE_RATE_HZ = 50.0;
    }

    public static class TurretConstants{
        public static final double kMinOutput = -1;
        public static final double kMaxOutput = 1;

        //CONFIGURACION DE MAXMOTION
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;

        public static final double kCruiseVelocity = 2.0;
        public static final double kMaxAcc = 4;

        //PEDIR
        public static final double kGearRatio = 15;

        public static final double kPositionFactor = 1 / kGearRatio; //Si NO ESTA DIRECTO A LA TORRETA
        //Si esta directo a la torreta usar (1:1):
        //public static final double kPositionFactor  = 1; 
        public static final double kVelocityFactor = kPositionFactor / 60; //RPM A RPS

        //------------VARIABLES DE VOLTAJE Y LIMITE DE CORRIENTE------------
        public static final int kCurrentLimit = 40;
        public static final double kMaxVolts = 12;

        //LÍMITE FÍSICO DE LA TORRETA, preguntar por si acaso (este valor me lo dió agus)
        public static final double kLowerLimit = Units.degreesToRotations(-200);
        public static final double kUpperLimit = Units.degreesToRotations(200);

        //PEDIR
        public static final int kMotorId = 0;

        public static final boolean kMotorInverted = false;
        public static final boolean kEncoderInverted = false;
    
        //Ubicación exacta del centro de la torreta respecto al robot:
        // X (Q metros): Significa que la torreta está Q cm hacia adelante del centro del robot.
        // Y (E metros): La torreta esta E metros en izquierda o derecha (0 centrada)
        // Z (W metros): El shooter está a W cm de altura del suelo.
        public static final Translation3d kMounting = new Translation3d(0.10, 0, 0.5);

        //VARIABLES DE SIMULACION (No las tienen que sacar), NO PASA NADA SI SE CAMBIAN, DEJARLAS TAL CUAL
        public static final Distance kRadius = Meters.of(0.15);
        public static final Mass kMass = Kilograms.of(5);

  
    }

    public static class IntakeConstants{
        public static final int ROLLERS_CAN_ID = 0;
        public static final int ANGULATOR_CAN_ID = 0;

        public static final double Tolerance = 1;
        public static final double GEAR_RATIO = 0;
    }
}