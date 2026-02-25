package frc.robot.Mechanisms.Outake.Arm;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stzteam.forgemini.io.IOSubsystem;
import com.stzteam.forgemini.io.Signal;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Mechanisms.MechanismsConstants.ArmConstants;


// Angulator Subsystem 
//+Interpolación Desde Mars
//+Forge Mini

    public class ArmSub extends IOSubsystem {
    
    private final TalonFX turretAngulator;
    private final MotionMagicExpoVoltage motionRequest;
    
    private double positionDegrees = 0.0;
    private Rotation2d rotation = new Rotation2d();
    private double targetAngle = 0.0;
    private double timestamp = 0.0;
    
  
    public ArmSub() {
        super("AngulatorSubsystem");
        
        // Inicialización del Kraken
        turretAngulator = new TalonFX(ArmConstants.kId, "Canivore");
        motionRequest = new MotionMagicExpoVoltage(0);
        
    
        configureMotor();
        configMotion();
        
        System.out.println("[AngulatorSub] Initialized - Usando INTERPOLATION_MAP directo desde Constants");
    }
    
    //Congiguración del motor
    private void configureMotor() {
        var motorConfigs = new MotorOutputConfigs();
        
        motorConfigs.Inverted = ArmConstants.INVERTED ? 
            com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive :
            com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        
        motorConfigs.NeutralMode = NeutralModeValue.Brake;
        
        var limitConfigs = new CurrentLimitsConfigs();
        limitConfigs.StatorCurrentLimit = ArmConstants.currentLimit;
        limitConfigs.StatorCurrentLimitEnable = true;
        
        turretAngulator.getConfigurator().apply(limitConfigs);
        turretAngulator.getConfigurator().apply(motorConfigs);
    }
    
    //Configuración del motion Magic versión expo
    private void configMotion() {
        var talonFXConfigs = new TalonFXConfiguration();
        var slot0Configs = talonFXConfigs.Slot0;
        
        slot0Configs.kS = ArmConstants.kS;
        slot0Configs.kV = ArmConstants.kV;
        slot0Configs.kA = ArmConstants.kA;
        slot0Configs.kP = ArmConstants.kP;
        slot0Configs.kI = ArmConstants.kI;
        slot0Configs.kD = ArmConstants.kD;
        
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ArmConstants.kCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = ArmConstants.kMaxAcc;
        
        turretAngulator.getConfigurator().apply(talonFXConfigs);
    }
    
    //Forge Mini telemetría    
    @Signal(key = "Position")
    public double getPositionSignal() {
        return positionDegrees;
    }
    
    @Signal(key = "TargetAngle")
    public double getTargetAngleSignal() {
        return targetAngle;
    }
    
    @Signal(key = "AtTarget")
    public boolean getAtTargetSignal() {
        return isAtTarget();
    }
    
    @Signal(key = "Rotation")
    public double getRotationDegreesSignal() {
        return rotation.getDegrees();
    }
    
    @Signal(key = "Velocity_RPS")
    public double getVelocitySignal() {
        return turretAngulator.getVelocity().getValueAsDouble();
    }
    
    @Signal(key = "Current")
    public double getCurrentSignal() {
        return turretAngulator.getSupplyCurrent().getValueAsDouble();
    }
    
    @Signal(key = "Temperature")
    public double getTemperatureSignal() {
        return turretAngulator.getDeviceTemp().getValueAsDouble();
    }
    
    //Control del métodos (ArmRequest)
    
  
    public void stopMotor() {
        applyOutput(0);
    }
    
    
    public void setPosition(double angleDegrees) {
        targetAngle = angleDegrees;
        turretAngulator.setControl(
            motionRequest.withPosition(Units.degreesToRotations(angleDegrees)).withSlot(0)
        );
    }
    
    //Interolación pasado igual desde arm request
    public void setPositionFromDistance(double distanceMeters) {
        Double interpolatedAngle = ArmConstants.INTERPOLATION_MAP.get(distanceMeters);
        
        if (interpolatedAngle == null) {
            System.out.println("[AngulatorSub] Distance out o range: " + distanceMeters + "m");
            stopMotor();
            return;
        }
        
        setPosition(interpolatedAngle);
    }
    

    public void applyOutput(double volts) {
        turretAngulator.setVoltage(volts);
    }
    

    public boolean isAtTarget() {
        return isAtTarget(ArmConstants.ARM_TOLERANCE);
    }
    

    public boolean isAtTarget(double toleranceDegrees) {
      
        return MathUtil.isNear(targetAngle, positionDegrees, toleranceDegrees);
    }
    

    public double getPositionDegrees() {
        return positionDegrees;
    }
    
    public double getTargetAngleDegrees() {
        return targetAngle;
    }
    
    public Rotation2d getRotation() {
        return rotation;
    }
    
    
    @Override
    public void periodicLogic() {
    //ArmIOKraken
        var rotorPosSignal = turretAngulator.getRotorPosition();
        var rotorPosRotations = rotorPosSignal.getValueAsDouble();
        
        
        positionDegrees = Units.rotationsToDegrees(rotorPosRotations);
        rotation = Rotation2d.fromRotations(rotorPosRotations);
        timestamp = rotorPosSignal.getTimestamp().getLatency();
        
        updateDashboard();
    }
    
  
    private void updateDashboard() {
        //Todos los angulos son en degrees
        SmartDashboard.putNumber("Angulator/Position", positionDegrees);
        SmartDashboard.putNumber("Angulator/TargetAngle", targetAngle);
        SmartDashboard.putNumber("Angulator/Rotation", rotation.getDegrees());
        
        SmartDashboard.putBoolean("Angulator/AtTarget", isAtTarget());
        SmartDashboard.putNumber("Angulator/Current", turretAngulator.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Angulator/Temperature", turretAngulator.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("Angulator/Timetamp", timestamp);
    }
    //Aquí en un futuro podremos quitar lo que no ocupemos
    @Override
    public void simulationPeriodic() {
        //Por si queremos hacer una simulación
    }
}