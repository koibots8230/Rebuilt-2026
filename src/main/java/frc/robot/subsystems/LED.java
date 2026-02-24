package frc.robot.subsystems;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LED extends SubsystemBase {
    private LEDMode defaultMode;
    private LEDMode autonMode;
    private LEDMode shift1Mode;
    private LEDMode shift2Mode;
    private LEDMode endGameMode;
    private boolean isBlue;
    private boolean isBlueActiveFirst;
    private boolean Active;
    private int shift;
    private int[] activeShifts;
    private String firstInactiveHub;
    private Double matchTime;

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
    public LED() {
        defaultMode = new LEDMode(0, "Default");
        autonMode = new LEDMode(1, "Auton");
        shift1Mode = new LEDMode(0, "Shift1");
        shift2Mode = new LEDMode(0, "Shift2");
        endGameMode = new LEDMode(0, "endGame");
        Optional<Alliance> ally = DriverStation.getAlliance();

        isBlue = ((ally.get() == Alliance.Blue) ? true : false);
        firstInactiveHub = "";
    }

    public Command setModeCommand(LEDMode mode){
        return Commands.runOnce(() -> mode.writeModeString());
    }

    public void initializeLogic(){
        firstInactiveHub = DriverStation.getGameSpecificMessage();

        if (firstInactiveHub.length() > 0){
            switch (firstInactiveHub.charAt(0)) {
                case 'R':
                    isBlueActiveFirst = true;
                    break;
                case 'B':
                    isBlueActiveFirst = false;
                    break;
                default:
                    System.out.println("Game data does not fit specified parameters");
                    break;
            }
        } else {
            System.out.println("There is no game data yet");
        }

        if (isBlueActiveFirst && isBlue) {
            activeShifts = new int[]{1, 3};
        } else if (isBlueActiveFirst && !isBlue){
            activeShifts = new int[]{2,4};
        } else if (!isBlueActiveFirst && isBlue){
            activeShifts = new int[]{2,4};
        } else {
            activeShifts = new int[]{1,3};
        }

    }

    @Override
    public void periodic(){
        matchTime = DriverStation.getMatchTime();
        
        initializeLogic();

        if (DriverStation.isAutonomous()){
            setModeCommand(autonMode);
        } else {
            setModeCommand(defaultMode);
        }
    }
}
