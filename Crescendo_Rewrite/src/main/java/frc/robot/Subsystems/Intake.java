package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import RockinLib.MotorControllers.RockinSparkMax;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    RockinSparkMax topMotor;
    RockinSparkMax bottomMotor;
    DigitalInput beamBreak;
    boolean flipBeamBreak;
    IntakeState state = IntakeState.IDLE;
    public Intake(){
        beamBreak = new DigitalInput(Constants.IntakeConstants.INTAKE_BEAMBREAK_ID);
        flipBeamBreak = false;
        topMotor = new RockinSparkMax(Constants.IntakeConstants.INTAKE_TOPROLLER_ID, MotorType.kBrushless);
        bottomMotor = new RockinSparkMax(Constants.IntakeConstants.INTAKE_BOTTOMROLLER_ID, MotorType.kBrushless);
        bottomMotor.follow(topMotor,true);
        topMotor.setCurrentLimit(40);
        bottomMotor.setCurrentLimit(40);
        
        
    }
    public static enum IntakeState{
        INTAKING(12),
        OUTTAKING(-12),
        HOLDING(0),
        IDLE(0);
        public final double voltage;
        private IntakeState(double voltage) {
            this.voltage = voltage;
        }
    }
    @Override
    public void periodic() {
        //if beam break triggered and not outtaking, set state to holding
        this.state = (beamBreak.get() == !flipBeamBreak && state != IntakeState.OUTTAKING) ? IntakeState.HOLDING : this.state;
        //if bam break not triggered and outtaking, set state to idle
        this.state = (beamBreak.get() == flipBeamBreak && state == IntakeState.OUTTAKING) ? IntakeState.IDLE : this.state;
        setSpeed(state.voltage);
        SmartDashboard.putString("Intake State", this.getState().toString());
        DogLog.log(" Intake State", this.state);
        SmartDashboard.putNumber("Intake Current", topMotor.getAppliedOutput());
    }
    private void setSpeed(double voltage){
        topMotor.setVoltage(voltage);
    }
    public Command setState(IntakeState state){
        return runOnce(() -> 
            this.state = state
        );
    }
 
    public IntakeState getState(){
        return this.state;
    }
    public boolean isHolding(){
        return this.state == IntakeState.HOLDING;
    }
    public boolean isIntaking(){
        return this.state == IntakeState.INTAKING;
    }
    
}
