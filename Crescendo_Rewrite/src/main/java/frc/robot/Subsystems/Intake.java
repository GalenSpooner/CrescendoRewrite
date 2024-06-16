package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import RockinLib.MotorControllers.RockinSparkMax;
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
    IntakeState state = IntakeState.NEUTRAL;
    public Intake(){
        beamBreak = new DigitalInput(Constants.IntakeConstants.INTAKE_BEAMBREAK_ID);
        flipBeamBreak = false;
        topMotor = new RockinSparkMax(Constants.IntakeConstants.INTAKE_TOPROLLER_ID, MotorType.kBrushless);
        bottomMotor = new RockinSparkMax(Constants.IntakeConstants.INTAKE_BOTTOMROLLER_ID, MotorType.kBrushless);
        bottomMotor.follow(topMotor,true);
        topMotor.setCurrentLimit(40);
        bottomMotor.setCurrentLimit(40);
        
        
    }
    private static enum IntakeState{
        INTAKING(12),
        OUTTAKING(12),
        HOLDING(0),
        NEUTRAL(0);
        public final double voltage;
        private IntakeState(double voltage) {
            this.voltage = voltage;
        }
    }
    @Override
    public void periodic() {
        this.state = (beamBreak.get() == !flipBeamBreak) ? IntakeState.HOLDING : this.getState();
        setSpeed(state.voltage);
        SmartDashboard.putString("Intake State", this.getState().toString());
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
        return state;
    }
}
