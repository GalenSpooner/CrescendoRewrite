package RockinLib.PID;
import java.time.chrono.HijrahChronology;

import com.ctre.phoenix6.hardware.CANcoder;

import RockinLib.MotorControllers.RockinSparkMax;
import RockinLib.MotorControllers.RockinTalon;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class RockinSparkPIDSubsystem extends SubsystemBase{
    public RockinSparkMax[] sparks;
    public PIDController pidController;
    public CANcoder encoder;
    private double pidOutput = 0;
    private double setpoint;
    /**
     * subsystembase but if it was good
     * Todo: "finish" and create custom cancoder wrapper
     * @param pidController
     * @param encoder
     * @param talons
     * Have your talons pre-configed
     * @param name
     
    **/
    public RockinSparkPIDSubsystem(String name, PIDController pidController, double initStartPoint, CANcoder encoder, RockinSparkMax... sparks){
        this.sparks = sparks;
        this.pidController = pidController;
        this.encoder = encoder;
        setpoint = initStartPoint;
        this.setName(name);
        
    }
    /**
     * Use this instead of the normal periodic method
     **/
    public void highLevelPeriodic(){

    }
    public void calcAndApply(double setpoint){
        for(int x = 0; x <= sparks.length; x++){
         sparks[x].set(pidController.calculate(encoder.getAbsolutePosition().getValueAsDouble(), setpoint));
         SmartDashboard.putNumber((this.getName() + " motor " + (x+1) + " PID value"),pidController.calculate(encoder.getAbsolutePosition().getValueAsDouble(), setpoint));
        }
    }
    public double getSetpoint(){
        return this.setpoint;
    }
    public void setSetpoint(double setpoint){
        this.setpoint = setpoint;
    }
   
    /** 
     * Do not override this method. Instead use {@link RockinSparkPIDSubsystem#highLevelPeriodic()}
     *
    */
     @Override 
    public void periodic(){
        calcAndApply(getSetpoint());
        highLevelPeriodic();
        
    }

}
