package frc.robot.utils.pneumatics;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class SmartDoubleSolenoid extends DoubleSolenoid {
    public Value lastSet = null;
    public SmartDoubleSolenoid(int forwardChannel, int reverseChannel) {
        super(forwardChannel, reverseChannel);
    }

    public SmartDoubleSolenoid(int moduleNumber, int forwardChannel, int reverseChannel) {
        super(moduleNumber, forwardChannel, reverseChannel);
    }

    public void forward() {
        if(lastSet != Value.kForward){
            set(Value.kForward);
            lastSet = Value.kForward;
        }
    }

    public void reverse() {
        if(lastSet != Value.kReverse){
            set(Value.kReverse);
            lastSet = Value.kReverse;
        }
    }

    public void off() {
        if(lastSet != Value.kOff){
            set(Value.kOff);
            lastSet = Value.kOff;
        }
    }

}
