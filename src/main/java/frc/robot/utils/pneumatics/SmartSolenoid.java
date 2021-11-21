package frc.robot.utils.pneumatics;

import edu.wpi.first.wpilibj.Solenoid;

public class SmartSolenoid extends Solenoid {
    public boolean lastSet = false;
    public SmartSolenoid(int channel) {
        super(channel);
    }

    public SmartSolenoid(int moduleNumber, int channel) {
        super(moduleNumber, channel);
    }

    public void extend(){
        if(!lastSet){
            set(true);
            lastSet = true;
        }
    }
    public void reverse(){
        if(lastSet){
            set(false);
            lastSet = false;
        }
    }
}
