package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.math.StateSpaceUtil;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;
import frc.robot.utils.motor.LazyTalonSRX;

public class ElevatorStateSpaceController {

    DCMotor motor = DCMotor.getNEO(0);
    LazyTalonSRX motorOb;
    double massKg = 0;
    double radiusMeters = 0.02; //driving drum radius
    double G = 1; //The reduction between motor and drum, as a ratio of output to input.

    double modelAccuracy1 = 0.2;
    double modelAccuracy2 = 0.2;
    double encoderAccuracy = 0.8;
    double loopTime = 1;

    double positionEpsilon = 1;
    double velocityEpsilon = 1;
    double maxVoltageOutput = 1;

    double maxVel = 1;
    double maxAcc = 1;


    LinearSystem<N2, N1, N1> ls; //<States, Inputs, Outputs>
    KalmanFilter<N2, N1, N1> observer;
    LinearQuadraticRegulator<N2, N1, N1> lqr; //<States, Inputs, Outputs>
    LinearSystemLoop<N2, N1, N1> loop; //<States, Inputs, Outputs>

    TrapezoidProfile.Constraints constraints;
    TrapezoidProfile.State lastState = new TrapezoidProfile.State();
    TrapezoidProfile.State goal;

    public ElevatorStateSpaceController(LazyTalonSRX srx){
        ls = LinearSystemId.createElevatorSystem(motor, massKg, radiusMeters, G);

        observer = new KalmanFilter<>(
                Nat.N2(), Nat.N1(),
                ls,
                VecBuilder.fill(modelAccuracy1, modelAccuracy2),
                VecBuilder.fill(encoderAccuracy),
                loopTime
        );

        lqr = new LinearQuadraticRegulator<>(
                ls,
                VecBuilder.fill(positionEpsilon, velocityEpsilon),
                VecBuilder.fill(maxVoltageOutput),
                loopTime
        );

        loop = new LinearSystemLoop<>(
                ls,
                lqr,
                observer,
                maxVoltageOutput,
                loopTime
        );

        constraints = new TrapezoidProfile.Constraints(maxVel, maxAcc);

        motorOb = srx;
    }

    public ElevatorStateSpaceController(
            DCMotor motor, double massKg, double radiusMeters, double G,

            double encoderAccuracy, double modelAccuracy1, double modelAccuracy2, double loopTime,

            double positionEpsilon, double velocityEpsilon, double maxVoltageOutput,

            double maxVel, double maxAcc
    ){

        ls = LinearSystemId.createElevatorSystem(motor, massKg, radiusMeters, G);

        observer = new KalmanFilter<>(
                Nat.N2(), Nat.N1(),
                ls,
                VecBuilder.fill(modelAccuracy1, modelAccuracy2),
                VecBuilder.fill(encoderAccuracy),
                loopTime
        );

        lqr = new LinearQuadraticRegulator<>(
                ls,
                VecBuilder.fill(positionEpsilon, velocityEpsilon),
                VecBuilder.fill(maxVoltageOutput),
                loopTime
        );

        loop = new LinearSystemLoop<>(
                ls,
                lqr,
                observer,
                maxVoltageOutput,
                loopTime
        );

        constraints = new TrapezoidProfile.Constraints(maxVel, maxAcc);
    }

    public void reset(double curEnc){
        loop.reset(VecBuilder.fill(curEnc, 0.0));
        lastState = new TrapezoidProfile.State(curEnc, 0.0);
    }

    public void setPosition(double targetPosition, double curEnc){
        goal = new TrapezoidProfile.State(targetPosition, 0);

        lastState = (new TrapezoidProfile(constraints, goal, lastState)).calculate(0.02);

        loop.setNextR(lastState.position, lastState.velocity);

        loop.correct(VecBuilder.fill(curEnc));

        loop.predict(0.02);

        double volt = loop.getU(0);

        motorOb.setVoltage(volt);

    }

}
