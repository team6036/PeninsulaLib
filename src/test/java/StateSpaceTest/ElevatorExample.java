package StateSpaceTest;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.IOException;

public class ElevatorExample extends SubsystemBase {

    WPI_TalonSRX elevator1, elevator2;
    double targetPosition = getPosition();
    ElevatorStateSpaceController mController;

    public ElevatorExample() throws IOException {

        elevator1 = new WPI_TalonSRX(1);
//        talon.configSelectedFeedbackSensor(CTRE_, 0, timeoutMillis);

        elevator2 = new WPI_TalonSRX(0);
        elevator2.set(ControlMode.Follower, 1);

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
