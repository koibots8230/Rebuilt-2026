package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.LEDConstants;

public class LED {
    public class LEDMode {
        private int modeString;
        private String modeName;
        private SerialPort uart;

        public LEDMode(int modeString, String modeName){
            this.modeString = modeString;
            this.modeName = modeName;
            this.uart = new SerialPort(LEDConstants.BAUD_RATE, SerialPort.Port.kMXP);
        }

        public void writeModeString(){
            uart.writeString(Integer.toString(modeString));
        }
    }
    
    public Command setModeCommand(LEDMode mode){
        return Commands.runOnce(() -> mode.writeModeString());
    }
}
