package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;

public class ElevatorStateSpaceController {

    DCMotor motor = DCMotor.getNEO(0);
    double massKg = 0;
    double radiusMeters = 0.02; //driving drum radius
    double G = 1; //The reduction between motor and drum, as a ratio of output to input.


    LinearSystem<N2, N1, N1> ls; //<States, Inputs, Outputs>
    KalmanFilter<N2, N1, N1> observer;
    LinearQuadraticRegulator<N2, N1, N1> lqr; //<States, Inputs, Outputs>
    LinearSystemLoop<N2, N1, N1> loop; //<States, Inputs, Outputs>

    public ElevatorStateSpaceController(){
        ls = LinearSystemId.createElevatorSystem(motor, massKg, radiusMeters, G);
    }

    public ElevatorStateSpaceController(
            DCMotor motor, double massKg, double radiusMeters, double G,

            double encoderAccuracy, double modelAccuracy1, double modelAccuracy2, double loopTime,

            double positionEpsilon, double velocityEpsilon, double maxVoltageOutput
    ){
        ls = LinearSystemId.createElevatorSystem(motor, massKg, radiusMeters, G);

        observer = new KalmanFilter<N2, N1, N1>(
                Nat.N2(), Nat.N1(),
                ls,
                VecBuilder.fill(modelAccuracy1, modelAccuracy2),
                VecBuilder.fill(encoderAccuracy),
                loopTime
        );

        lqr = new LinearQuadraticRegulator<N2, N1, N1>(
                ls,
                VecBuilder.fill(positionEpsilon, velocityEpsilon),
                VecBuilder.fill(maxVoltageOutput),
                loopTime
        );

        loop = new LinearSystemLoop<N2, N1, N1>(
                ls,
                lqr,
                observer,
                maxVoltageOutput,
                loopTime
        );
    }

    public void setPosition(double targetPosition){

    }
}
