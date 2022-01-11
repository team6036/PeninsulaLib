package frc.robot.subsystems.stateSpaceControllers;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;

public class FlywheelStateSpaceController {
    public final DCMotor right = DCMotor.getNEO(2);

    public final double momentOfInertia = 0.00032;
    public final double flywheelGearing = 1.0;

    public final double modelAccuracy = 3.0;
    public final double encoderAccuracy = 3.0;

    public final double loopSecond = 0.020;

    public final double velocityErrorTolerance = 8.0;
    public final double controlVoltageTolerance = 12.0;

    public final double maxVoltageVolts = 12.0;

    public final int encoderA = 0, encoderB = 1;

    public int spinUpRadiansPerSecond = 350;


    public final LinearSystem<N1, N1, N1> m_flywheelPlant =
            LinearSystemId.createFlywheelSystem(
                    right, momentOfInertia, flywheelGearing
            );

    public final KalmanFilter<N1, N1, N1> m_observer =
            new KalmanFilter<>(
                    Nat.N1(),
                    Nat.N1(),
                    m_flywheelPlant,
                    VecBuilder.fill(modelAccuracy),
                    VecBuilder.fill(encoderAccuracy),
                    loopSecond
            );

    public final LinearQuadraticRegulator<N1, N1, N1> m_controller =
            new LinearQuadraticRegulator<>(
              m_flywheelPlant,
              VecBuilder.fill(velocityErrorTolerance),
              VecBuilder.fill(controlVoltageTolerance),
              loopSecond
            );

    public final LinearSystemLoop<N1, N1, N1> m_loop =
            new LinearSystemLoop<>(
                    m_flywheelPlant,
                    m_controller,
                    m_observer,
                    maxVoltageVolts,
                    loopSecond
            );

    public final Encoder m_encoder = new Encoder(encoderA, encoderB); //encoder a channel, b channel

    public double update(){
        m_loop.setNextR(VecBuilder.fill(spinUpRadiansPerSecond)); //spin-up rad per second

        m_loop.correct(VecBuilder.fill(m_encoder.getRate())); //encoder get rate

        m_loop.predict(loopSecond);

        double nextVoltageSet = m_loop.getU(0);

        return nextVoltageSet;

    }

}
