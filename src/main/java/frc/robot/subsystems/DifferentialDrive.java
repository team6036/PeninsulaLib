package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.driveControllers.ArcadeDrive;
import frc.robot.subsystems.driveControllers.DriveControllerBase;
import frc.robot.utils.motor.LazyTalonSRX;
import frc.robot.utils.motor.MotorSetupUtility;

public class DifferentialDrive extends SubsystemBase {

    LazyTalonSRX right1;
    LazyTalonSRX right2;

    LazyTalonSRX left1;
    LazyTalonSRX left2;

    DriveControllerBase driveController = new ArcadeDrive();
//    DifferentialDriveOdometry odo = new DifferentialDriveOdometry();

    public DifferentialDrive(){
        right1 = new LazyTalonSRX(0);
        left1 = new LazyTalonSRX(2);

        right2 = MotorSetupUtility.slaveLazyTalonSRX(1, false, right1.getDeviceID());
        left2 = MotorSetupUtility.slaveLazyTalonSRX(3, false, left1.getDeviceID());
    }

    @Override
    public void periodic(){

    }
}
