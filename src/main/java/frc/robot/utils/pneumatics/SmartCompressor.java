package frc.robot.utils.pneumatics;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Timer;

public class SmartCompressor extends Compressor{
    public SmartCompressor(int module) {
        super(module);
    }

    public void timeCharge(double time){
        start();
        Timer.delay(time);
        stop();
    }

    public boolean fullCharge(){
        if(!getPressureSwitchValue()){
            start();
            return false;
        }else{
            stop();
            return true;
        }
    }

}
