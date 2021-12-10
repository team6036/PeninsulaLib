package StateSpaceTest;

import org.junit.Test;

import java.io.IOException;
//import org.junit.jupiter.api.BeforeAll;
//import org.junit.jupiter.api.Test;


public class Tester {
    @Test
    public void main() throws IOException {
//        LazyTalonSRX d = new LazyTalonSRX(0);
//        WPI_TalonSRX d = new WPI_TalonSRX(1);
//        System.out.println(32879);
        ElevatorStateSpaceController df = new ElevatorStateSpaceController();
        df.reset(4);
        df.debugPosition(30, 4);
    }
}
