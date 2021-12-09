package frc.robot.utils.motor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class LazyTalonSRX extends WPI_TalonSRX {
    double mLastSet = Double.NaN;
    ControlMode mLastControl = null;
    /**
     * Constructor for LazyTalonSRX object
     *
     * @param deviceNumber CAN Device ID of Device
     */
    public LazyTalonSRX(int deviceNumber) {
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
