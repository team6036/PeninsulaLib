package frc.robot.subsystems.driveControllers;

public abstract class DriveControllerBase {
    public DriveControllerBase(){

    }
    public abstract DriveValue getPowers(double right, double left);

    public static class DriveValue{
        double right, left;

        public DriveValue(double right, double left) {
            this.right = right;
            this.left = left;
        }

        public double getRight() {
            return right;
        }

        public void setRight(double right) {
            this.right = right;
        }

        public double getLeft() {
            return left;
        }

        public void setLeft(double left) {
            this.left = left;
        }
    }
}
