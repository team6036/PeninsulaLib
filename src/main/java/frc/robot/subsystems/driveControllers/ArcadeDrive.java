package frc.robot.subsystems.driveControllers;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

public class ArcadeDrive extends DriveControllerBase{

    @Override
    public DriveValue getPowers(double right, double left) {
        return new DriveValue(0, 0);
    }
}
