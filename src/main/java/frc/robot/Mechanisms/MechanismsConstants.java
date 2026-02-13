package frc.robot.Mechanisms;


public class MechanismsConstants {
    
    //Constantes del Index
    public static class IndexConstants {
        
        // IDs de los 2 motores, rodillos e index
        public static final int ROLLERS_MOTOR_CAN_ID = 20;
        public static final int INDEX_MOTOR_CAN_ID = 21;
        
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
}