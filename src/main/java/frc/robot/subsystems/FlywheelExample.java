package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.stateSpaceControllers.FlywheelStateSpaceController;

public class FlywheelExample extends SubsystemBase {

    CANSparkMax right, left;
    int targetRpm = 0;
    FlywheelStateSpaceController m_controller  = new FlywheelStateSpaceController();

    public FlywheelExample(){
        right = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        left = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
        left.setInverted(true);
    }

    @Override
    public void periodic() {
        double voltNew = m_controller.update();

        m_controller.spinUpRadiansPerSecond = targetRpm;

        right.setVoltage(voltNew);
        left.setVoltage(voltNew);

    }



}
