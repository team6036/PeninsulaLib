package frc.robot.utils;

import frc.robot.utils.motor.LazyTalonFX;
import frc.robot.utils.motor.LazyTalonSRX;

public class Watchdog {
    public static String watchSRX(LazyTalonSRX... SRXes){
        StringBuilder status = new StringBuilder("#          Temperature          Voltage");

        for(LazyTalonSRX talon: SRXes) line(status, talon.getDeviceID(), talon.getTemperature(), talon.getBusVoltage());

        return status.toString();
    }
    public static String watchFX(LazyTalonFX... FXes){
        StringBuilder status = new StringBuilder("#          Temperature          Voltage");

        for(LazyTalonFX talon: FXes) line(status, talon.getDeviceID(), talon.getTemperature(), talon.getBusVoltage());

        return status.toString();
    }

    private static void line(StringBuilder status, int deviceID, double temperature, double busVoltage) {
        status.append("\n");
        status.append(deviceID);
        status.append(space(9 + ((deviceID < 10) ? 1 : 0)));
        status.append(temperature);
        status.append(space(21 - (temperature + "").length()));
        status.append(busVoltage);
    }

    public static String space(int space){
        String y = "";
        for(int i = 0;i<space;i++) y += space;
        return y;
    }
}
