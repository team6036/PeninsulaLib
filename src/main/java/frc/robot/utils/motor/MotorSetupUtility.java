package frc.robot.utils.motor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class MotorSetupUtility {
    public static LazyTalonSRX lazyTalonSRX(int deviceNumber, FeedbackDevice feedbackDevice, double P, double I, double D, int timeoutMillis, boolean inverted){
        LazyTalonSRX talon = new LazyTalonSRX(deviceNumber);
        talon.configSelectedFeedbackSensor(feedbackDevice, 0, timeoutMillis);
        talon.config_kP(0, P, timeoutMillis);
        talon.config_kI(0, I, timeoutMillis);
        talon.config_kD(0, D, timeoutMillis);

        talon.setInverted(inverted);

        return talon;
    }

    public static LazyTalonSRX slaveLazyTalonSRX(int deviceNumber, boolean inverted, int masterId){
        LazyTalonSRX talon = new LazyTalonSRX(deviceNumber);
        talon.set(ControlMode.Follower, masterId);
        talon.setInverted(inverted);

        return talon;
    }


    public static LazyTalonFX lazyTalonFX(int deviceNumber, FeedbackDevice feedbackDevice, double F, double P, double I, double D, int timeoutMillis, boolean inverted){
        LazyTalonFX talon = new LazyTalonFX(deviceNumber);
        talon.configFactoryDefault();

        talon.configSelectedFeedbackSensor(feedbackDevice, 0, timeoutMillis);
        talon.config_kF(0, F, timeoutMillis);
        talon.config_kP(0, P, timeoutMillis);
        talon.config_kI(0, I, timeoutMillis);
        talon.config_kD(0, D, timeoutMillis);

        talon.setInverted(inverted);

        return talon;
    }


}
