package frc.robot.subsystems.stateSpaceControllers;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;
//import org.junit.jupiter.api.BeforeAll;
//import org.junit.jupiter.api.BeforeEach;
//import org.junit.jupiter.api.Test;


public class ElevatorStateSpaceController {

    DCMotor motor = DCMotor.getNEO(2);
    WPI_TalonSRX motorOb;
    double massKg = 1;
    double radiusMeters = 0.02; //driving drum radius
    double G = 1.0; //The reduction between motor and drum, as a ratio of output to input.

    double modelAccuracy1 = 0.2;
    double modelAccuracy2 = 0.2;
    double encoderAccuracy = 3.0;
    double loopTime = 0.02;

    double positionEpsilon = 0.2;
    double velocityEpsilon = 0.4;
    double maxVoltageOutput = 12.0;

    double maxVel = 1;
    double maxAcc = 1;


    LinearSystem<N2, N1, N1> plant; //<States, Inputs, Outputs>
    KalmanFilter<N2, N1, N1> observer;
    LinearQuadraticRegulator<N2, N1, N1> lqr; //<States, Inputs, Outputs>
    LinearSystemLoop<N2, N1, N1> loop; //<States, Inputs, Outputs>

    TrapezoidProfile.Constraints constraints;
    TrapezoidProfile.State lastState = new TrapezoidProfile.State();
    TrapezoidProfile.State goal;

    public ElevatorStateSpaceController(){

        plant = LinearSystemId.createElevatorSystem(motor, massKg, radiusMeters, G);

        observer = new KalmanFilter<>(
                Nat.N2(),
                Nat.N1(),
                plant,
                VecBuilder.fill(modelAccuracy1, modelAccuracy2),
                VecBuilder.fill(encoderAccuracy),
                loopTime
        );

        lqr = new LinearQuadraticRegulator<>(
                plant,
                VecBuilder.fill(positionEpsilon, velocityEpsilon),
                VecBuilder.fill(maxVoltageOutput),
                loopTime
        );

        loop = new LinearSystemLoop<>(
                plant,
                lqr,
                observer,
                maxVoltageOutput,
                loopTime
        );

        constraints = new TrapezoidProfile.Constraints(maxVel, maxAcc);

    }

    public ElevatorStateSpaceController(
            DCMotor motor, double massKg, double radiusMeters, double G,

            double encoderAccuracy, double modelAccuracy1, double modelAccuracy2, double loopTime,

            double positionEpsilon, double velocityEpsilon, double maxVoltageOutput,

            double maxVel, double maxAcc, WPI_TalonSRX srx
    ){

        plant = LinearSystemId.createElevatorSystem(motor, massKg, radiusMeters, G);

        observer = new KalmanFilter<>(
                Nat.N2(), Nat.N1(),
                plant,
                VecBuilder.fill(modelAccuracy1, modelAccuracy2),
                VecBuilder.fill(encoderAccuracy),
                loopTime
        );

        lqr = new LinearQuadraticRegulator<>(
                plant,
                VecBuilder.fill(positionEpsilon, velocityEpsilon),
                VecBuilder.fill(maxVoltageOutput),
                loopTime
        );

        loop = new LinearSystemLoop<>(
                plant,
                lqr,
                observer,
                maxVoltageOutput,
                loopTime
        );

        constraints = new TrapezoidProfile.Constraints(maxVel, maxAcc);

        motorOb = srx;
    }

    public void reset(double curEnc){
        loop.reset(VecBuilder.fill(curEnc, 0.0));
        lastState = new TrapezoidProfile.State(curEnc, 0.0);
    }

    /**
     *
     * @param targetPosition the target encoder position
     * @param curEnc the current encoder position
     */
    public void setPosition(double targetPosition, double curEnc){
        goal = new TrapezoidProfile.State(targetPosition, 0);

        lastState = (new TrapezoidProfile(constraints, goal, lastState)).calculate(0.02);

        loop.setNextR(lastState.position, lastState.velocity);

        loop.correct(VecBuilder.fill(curEnc));

        loop.predict(0.02);

        double volt = loop.getU(0);

        motorOb.setVoltage(volt);

    }

    public void debugPosition(double targetPosition, double curEnc){
        goal = new TrapezoidProfile.State(targetPosition, 0);

        lastState = (new TrapezoidProfile(constraints, goal, lastState)).calculate(0.02);

        loop.setNextR(lastState.position, lastState.velocity);

        loop.correct(VecBuilder.fill(curEnc));

        loop.predict(0.02);

        double volt = loop.getU(0);

        System.out.println("Volt: " + volt);
        System.out.println("Error: " + loop.getError());
        System.out.println("Last: " + lastState.position);
        System.out.println("Goal" + goal.position);
        System.out.println((new TrapezoidProfile(constraints, goal, lastState)));

    }

}
