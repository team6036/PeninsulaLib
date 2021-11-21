package frc.robot.utils.pneumatics;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Solenoid;

public class PneumaticControlExample {
    int portCom = 0;
    SmartCompressor com = new SmartCompressor(portCom);

    int portPor = 0, scale, offset;
    AnalogPotentiometer pressureTransducer = new AnalogPotentiometer(portPor, scale, offset);

    int portSolSingle = 0;
    Solenoid solenoid = new Solenoid(portSolSingle);

    int portSolDouble1 = 0, portSolDouble2 = 1;
    SmartDoubleSolenoid doubleSolenoid = new SmartDoubleSolenoid(portSolDouble1, portSolDouble2);

    public void main(String[] args) {
        doubleSolenoid.forward();
        doubleSolenoid.off();
        doubleSolenoid.reverse();

        solenoid.set(true);
        solenoid.set(false);

        pressureTransducer.get();

        com.start();
        com.stop();
        com.timeCharge(1);
        com.fullCharge();
    }

}
