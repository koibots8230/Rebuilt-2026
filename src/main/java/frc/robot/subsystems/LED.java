package frc.robot.subsystems;

import java.util.Optional;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LED extends SubsystemBase {
    private LEDMode defaultMode;
    private LEDMode shiftActiveMode;
    private LEDMode shiftInactiveMode;
    private LEDMode endGameMode;
    private boolean isBlue;
    private String shift;
    private String firstInactiveHub;
    private Double matchTime;
    private record shiftData(int Shift1, int Shift2){};
    private shiftData activeShifts;
    private static SerialPort serialPort = new SerialPort(LEDConstants.BAUD_RATE, SerialPort.Port.kOnboard);

    public class LEDMode {
        private int modeString;
        private String modeName;
        private byte[] modeByte;

        public LEDMode(int modeString, String modeName){
            this.modeString = modeString;
            this.modeName = modeName;
            this.modeByte = new byte[]{(byte) modeString};
        }

        public void writeModeString(){
            serialPort.write(modeByte, modeByte.length);
        }
    }

    public LED() {
        defaultMode = new LEDMode(0, "Default");
        shiftActiveMode = new LEDMode(2, "ShiftActive");
        shiftInactiveMode = new LEDMode(3, "ShiftInactive");
        endGameMode = new LEDMode(4, "EndGame");
        Optional<Alliance> ally = DriverStation.getAlliance();

        isBlue = ((ally.get() == Alliance.Blue) ? true : false);
        firstInactiveHub = "";
        shift = "";
    }

    public void configureLogic(){
        firstInactiveHub = DriverStation.getGameSpecificMessage();

        if (firstInactiveHub.length() > 0){
            switch (firstInactiveHub.charAt(0)) {
                case 'R':
                    if (isBlue){
                        activeShifts = new shiftData(1, 3);
                    } else {
                        activeShifts = new shiftData(2, 4);
                    }
                    break;
                case 'B':
                    if (isBlue){
                        activeShifts = new shiftData(2, 4);
                    } else {
                        activeShifts = new shiftData(1, 3);
                    }
                    break;
                default:
                    System.out.println("Game data does not fit specified parameters");
                    break;
            }
        } else {
            System.out.println("There is no game data yet");
        }
    }

    @Override
    public void periodic(){
        matchTime = DriverStation.getMatchTime();

        if(DriverStation.isTeleop()){
            if (matchTime >= 130){
                shift = "Transition";
            } else if (matchTime >= 105 && matchTime < 130){
                shift = "Shift 1";
            } else if (matchTime >= 80 && matchTime < 105){
                shift = "Shift 2";
            } else if (matchTime >= 55 && matchTime < 80){
                shift = "Shift 3";
            } else if (matchTime >= 30 && matchTime < 55){
                shift = "Shift 4";
            } else if (matchTime < 30){
                shift = "End Game";
            }

            switch (shift){
                case "Transition":
                    defaultMode.writeModeString();
                    break;
                case "Shift 1":
                    if (activeShifts.Shift1() == 1){
                        shiftActiveMode.writeModeString();
                    } else {
                        shiftInactiveMode.writeModeString();
                    } break;
                case "Shift 2":
                    if (activeShifts.Shift1() == 2){
                        shiftActiveMode.writeModeString();
                    } else {
                        shiftInactiveMode.writeModeString();
                    } break;
                case "Shift 3":
                    if (activeShifts.Shift2() == 3){
                        shiftActiveMode.writeModeString();
                    } else {
                        shiftInactiveMode.writeModeString();
                    } break;
                case "Shift 4":
                    if (activeShifts.Shift2() == 4){
                        shiftActiveMode.writeModeString();
                    } else {
                        shiftInactiveMode.writeModeString();
                    } break;
                case "End Game":
                    endGameMode.writeModeString();
                    break;
                default:
                    System.out.println("Shift data does not fit specified parameters or does not exist");
                    defaultMode.writeModeString();
                    break;
            }
        }
    }
    public Command setModeCommand(LEDMode mode){
        return Commands.runOnce(() -> mode.writeModeString());
    }

    public Command configureLogicCommand(){
        return Commands.runOnce(() -> configureLogic());
    }
}
