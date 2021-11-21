package frc.robot.utils.motor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class LazyTalonFX extends WPI_TalonFX {
    double mLastSet = Double.NaN;
    ControlMode mLastControl = null;
    /**
     * Constructor for LazyFX motor controller
     *
     * @param deviceNumber device ID of motor controller
     */
    public LazyTalonFX(int deviceNumber) {
        super(deviceNumber);
    }

    @Override
    public void set(ControlMode mode, double val){
        if(val != mLastSet || mode != mLastControl){
            mLastControl = mode;
            mLastSet = val;
            super.set(mode, val);
        }
    }
}
