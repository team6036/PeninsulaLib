package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.stateSpaceControllers.ElevatorStateSpaceController;
import frc.robot.utils.motor.LazyTalonSRX;
import frc.robot.utils.motor.MotorSetupUtility;

public class ElevatorExample extends SubsystemBase {

    LazyTalonSRX elevator1, elevator2;
    double targetPosition = getPosition();
    ElevatorStateSpaceController mController;

    public ElevatorExample() {

        elevator1 = MotorSetupUtility.lazyTalonSRX(
                0,
                FeedbackDevice.CTRE_MagEncoder_Relative,
                0.125,
                0.0,
                0.0,
                10,
                false
        );

        elevator2 = MotorSetupUtility.slaveLazyTalonSRX(
                1,
                true,
                elevator1.getDeviceID()
        );

        mController = new ElevatorStateSpaceController();
        mController.reset(getPosition());

    }

    public void setPosition(int position) {
        targetPosition = position;
    }

    public void setEncoderTickPosition() {
        elevator1.set(ControlMode.Position, targetPosition);
    }

    public void setStateSpacePosition() {
        mController.setPosition(targetPosition, getPosition());
    }

    public double getPosition(){
        assert elevator1 != null;
        return elevator1.getSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {
        setEncoderTickPosition();
        // This method will be called once per scheduler run
    }

}
