package RockinLib.Control;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

//used to wrap inputs from our guitar hero controller into what they appear as on the guitar
//will be updated as we make more of the buttons actually work (joystick, whammy bar)
public class RockinGuitar extends CommandJoystick{
    public RockinGuitar(int port){
        super(port);
    }
    public Trigger green(){
        return this.button(0);
    }
    public Trigger red(){
        return this.button(1);
    }
    public Trigger yellow(){
        return this.button(2);
    }
    public Trigger blue(){
        return this.button(4);
    }
    public Trigger orange(){
        return this.button(5);
    }
    public Trigger a(){
        return this.button(10);
    }
    public Trigger b(){
        return this.button(11);
    }


    
}
