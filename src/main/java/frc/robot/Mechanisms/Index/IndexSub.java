package frc.robot.Mechanisms.Index;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase; 
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.stzteam.forgemini.io.IOSubsystem;
import com.stzteam.forgemini.io.Signal;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Mechanisms.MechanismsConstants.IndexConstants;


public class IndexSub extends IOSubsystem {
    
    //Hardware
    private final SparkMax rollersMotor;
    private final RelativeEncoder rollersEncoder;
    
    private final SparkMax indexMotor;
    private final RelativeEncoder indexEncoder;
    
    // Sensor
    private final DigitalInput gamePieceSensor;
    private final boolean hasSensor;
    
    // Estado para ver si esta corriendo
    private boolean isRunning = false;  
  
    private double rollersTemperature = 0.0;
    private double indexTemperature = 0.0;
    private boolean hasGamePiece = false;
    
    //Constructor
    public IndexSub() {
        super("IndexSubsystem");
        
        
        rollersMotor = new SparkMax(IndexConstants.ROLLERS_MOTOR_CAN_ID, MotorType.kBrushless);
        indexMotor = new SparkMax(IndexConstants.INDEX_MOTOR_CAN_ID, MotorType.kBrushless);
        
        //Configuramos los motores
        configureMotor(rollersMotor, IndexConstants.ROLLERS_INVERTED);
        configureMotor(indexMotor, IndexConstants.INDEX_INVERTED);
        
        //Encoders
        rollersEncoder = rollersMotor.getEncoder();
        indexEncoder = indexMotor.getEncoder();
        
        //Si hay sensor inicializar
        hasSensor = IndexConstants.HAS_GAME_PIECE_SENSOR;
        if (hasSensor) {
            gamePieceSensor = new DigitalInput(IndexConstants.GAME_PIECE_SENSOR_DIO_PORT);
        } else {
            gamePieceSensor = null;
        }
        
        System.out.println("[IndexSub] Initialized - Simple ON/OFF control");
    }
    
    
    private void configureMotor(SparkMax motor, boolean inverted) {
        SparkMaxConfig config = new SparkMaxConfig();
        
        config.inverted(inverted);
        
        int periodMs = (int)(1000.0 / IndexConstants.TELEMETRY_UPDATE_RATE_HZ);
        config.signals.primaryEncoderVelocityPeriodMs(periodMs);
        config.signals.primaryEncoderPositionPeriodMs(periodMs);
        
    //Configuración
    //No se que es el error de depracated, En un futuro seguramente eso ya no funcione pero por ahroa si
    motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);


    }
    
    // TELEMETRÍA
    
    @Signal(key = "Running")
    public boolean getRunningSignal() {
        return isRunning;
    }
    
    @Signal(key = "HasGamePiece")
    public boolean getHasGamePieceSignal() {
        return hasGamePiece;
    }
    
    @Signal(key = "Rollers/Velocity_RPM")
    public double getRollersVelocitySignal() {
        return rollersEncoder.getVelocity();
    }
    
    @Signal(key = "Rollers/Temperature_C")
    public double getRollersTemperatureSignal() {
        return rollersTemperature;
    }
    
    @Signal(key = "Index/Velocity_RPM")
    public double getIndexVelocitySignal() {
        return indexEncoder.getVelocity();
    }
    
    @Signal(key = "Index/Temperature_C")
    public double getIndexTemperatureSignal() {
        return indexTemperature;
    }
    
    @Signal(key = "BusVoltage")
    public double getBusVoltageSignal() {
        return rollersMotor.getBusVoltage();
    }
    
    
  //Metodos de control
    public void Run(double speed) {
        isRunning = true;
        rollersMotor.set(speed);
        indexMotor.set(speed);
    }

    public void Runindex(){

        isRunning = true;
        indexMotor.set(IndexConstants.INDEXER_SPEED);
    }

    public void Runrollers(){

        isRunning = true;
        rollersMotor.set(IndexConstants.INDEXER_SPEED);
        
    }
  
    public void stop() {
        isRunning = false;
        rollersMotor.set(0.0);
        indexMotor.set(0.0);
    }
    
   
    public boolean isRunning() {
        return isRunning;
    }
    
   
    public boolean hasGamePiece() {
        if (hasSensor && gamePieceSensor != null) {
            return !gamePieceSensor.get();  
        }
        return hasGamePiece;
    }
    
    //Velocidad de los rodillos
    public double getRollersVelocityRPM() {
        return rollersEncoder.getVelocity();
    }
    
 //Velocidad del index
    public double getIndexVelocityRPM() {
        return indexEncoder.getVelocity();
    }
 
    public double getRollersTemperature() {
        return rollersTemperature;
    }
    
  
    public double getIndexTemperature() {
        return indexTemperature;
    }
    
    
    // Periodic
    
    @Override
    public void periodicLogic() {
        if (hasSensor) {
            hasGamePiece = hasGamePiece();
        }
        
       
        rollersTemperature = rollersMotor.getMotorTemperature();
        
      
        indexTemperature = indexMotor.getMotorTemperature();
        
        updateDashboard();
    }
    
  
    private void updateDashboard() {
        SmartDashboard.putBoolean("Indexer/Running", isRunning);
        SmartDashboard.putBoolean("Indexer/HasGamePiece", hasGamePiece);
        
        SmartDashboard.putNumber("Indexer/Rollers/Velocity_RPM", getRollersVelocityRPM());
        SmartDashboard.putNumber("Indexer/Rollers/Temperature_C", rollersTemperature);
        
        SmartDashboard.putNumber("Indexer/Index/Velocity_RPM", getIndexVelocityRPM());
        SmartDashboard.putNumber("Indexer/Index/Temperature_C", indexTemperature);
    }
    //Después se puede borrar la temperatura y otras cosas para simplificar la smartdashboard
    //initSendable? = https://docs.wpilib.org/en/stable/docs/software/telemetry/writing-sendable-classes.html
    @Override
    public void initSendable(SendableBuilder builder) {

        super.initSendable(builder);
        builder.setSmartDashboardType("Indexer");
        
        builder.addBooleanProperty("Running", this::isRunning, null);
        builder.addBooleanProperty("Has Game Piece", this::hasGamePiece, null);
        
        builder.addDoubleProperty("Rollers Velocity RPM", this::getRollersVelocityRPM, null);
        builder.addDoubleProperty("Rollers Temperature (C)", this::getRollersTemperature, null);
        
        builder.addDoubleProperty("Index Velocity RPM", this::getIndexVelocityRPM, null);
        builder.addDoubleProperty("Index Temperature (C)", this::getIndexTemperature, null);
    }
    

    }
