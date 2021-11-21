package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.controller.PIDController;

public class PeninsulaUtil{
    public static String PIDInfo(PIDController controller){
        String out = "";
        out += "P: " + controller.getP() + "\n";
        out += "I: " + controller.getI() + "\n";
        out += "D: " + controller.getD() + "\n";
        out += "\n";
        out += "P_err: " + controller.getPositionError() + "\n";
        out += "V_err: " + controller.getVelocityError() + "\n";
        out += "G_set: " + controller.getSetpoint() + "\n";
        return out;
    }

    public static boolean isStuck(double stuckCurrent, CANSparkMax motor){
        return motor.getOutputCurrent()>stuckCurrent;
    }

    public static boolean isStuck(double stuckCurrent, TalonFX motor){
        return motor.getOutputCurrent()>stuckCurrent;
    }

    public static boolean isStuck(double stuckCurrent, TalonSRX motor){
        return motor.getOutputCurrent()>stuckCurrent;
    }

}
